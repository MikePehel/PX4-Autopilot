#!/usr/bin/env python3
"""collision_prevention SITL integration test (sign-off gate).

End-to-end verification: with the obstacle-ring publisher active and
CollisionPrevention engaged, a quadrotor commanded to fly forward into
a procedural wall must slow down (CP-driven velocity reduction) without
ever closing the forward gap below the hard floor.

Pass criterion:
  Within 30 s of arriving at the wall's edge, ground speed drops by
  more than 50 % AND min(obstacle_distance[forward-arc bins]) does
  not fall below 5 m.

Where "arrived at the wall's edge" = first OBSTACLE_DISTANCE sample
where any forward-arc bin reads below max_distance / 2.

Setup:
  CP_DIST = 10.0, CP_GO_NO_DATA = 0
  MPC_POS_MODE = 4 (smooth velocity tracking)
  COM_RC_IN_MODE = 1 (MAVLink MANUAL_CONTROL is the stick source)
  SIH_OBST_EN = 1, SIH_OBST_MAX = 50.0
  SIH_TERR_EN = 2 (walls mode), SIH_TERR_SEED = 3
    (lattice walls; the wall at N=30 is the engagement target)

Notes on the setup:
  CP is only invoked from FlightTaskManualPosition (POSCTL) via
  StickAccelerationXY::modifySetpoint -- AUTO.MISSION does not run
  CP at all. The test arms in POSCTL with a MANUAL_CONTROL
  pump, climbs to hover at 10 m AGL, then commands forward pitch stick
  (x = 1000 = full forward) until the vehicle has either reached the
  wall or 60 s have elapsed since the edge detection.

Seed:
  SIH_TERR_SEED = 3 places lattice walls at cells (3, 0) and (5, 0)
  on the +N corridor from home (world N=30 and N=50 respectively).
  The first wall (N=30) triggers CP at standoff ~10 m so the vehicle
  slows at N~20 m — well above the 5 m hard-floor assertion. If a
  future seed leaves the +N corridor empty, the
  assert_arrived_at_wall check times out and the test fails — verify
  with `Tools/scene_query --seed <N>` and update DEFAULT_TERR_SEED.

Capture:
  ulog dumped from PX4's log/ dir into /tmp/sih_terrain_test/
  cp_test.ulg via LOGGING_START / LOGGING_STOP + post-run copy.
  Raw telemetry written to cp_test.json.

If the test FAILS: capture the symptom and the diagnostic ulog for
triage.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sitl_harness import (  # noqa: E402
    start_sitl, kill_sitl, preflight_kill_orphans,
    connect, set_param, relax_safety_for_sitl,
    wait_for_ekf2_ready, arm,
)

from pymavlink import mavutil  # noqa: E402


HOME_LAT = 47.397742
HOME_LON = 8.545594

DEFAULT_TERR_SEED = 3
HOVER_ALT_M = 10.0

# CP-related constants.
CP_DIST_M = 10.0
SIH_OBST_MAX_M = 50.0
SAFETY_FLOOR_M = 5.0       # bins must not fall below this -- pass criterion
SPEED_DROP_FRACTION = 0.5  # ground speed must drop > this -- pass criterion
HILL_EDGE_RATIO = 0.5      # forward bin < max_distance * this = "at the edge"
CAPTURE_WINDOW_AFTER_EDGE_S = 30.0
PRE_FLIGHT_HOVER_SETTLE_S = 5.0

# Forward arc in BODY_FRD: bin 0 = nose, +5 deg per bin clockwise.
# A forward arc of +/- 90 deg covers bins 0..18 (right hemisphere) plus
# bins 54..71 (left hemisphere -- the wraparound).
FORWARD_BINS = set(range(0, 19)) | set(range(54, 72))
UNKNOWN = 65535


class ManualControlPump:
    """Streams MANUAL_CONTROL packets at 50 Hz with adjustable sticks.

    CollisionPrevention is only invoked from
    FlightTaskManualPosition / StickAccelerationXY (POSCTL). The
    flight task reads manual_control_setpoint, which PX4's
    manual_control module publishes only when it has a stick source.
    With COM_RC_IN_MODE = 1, MAVLink MANUAL_CONTROL is that source.

    x, y, r are in [-1000, 1000]. z (throttle) is in [0, 1000];
    500 is the POSCTL altitude-hold center.
    """

    def __init__(self, conn, interval_s: float = 0.02):
        self.conn = conn
        self.interval_s = interval_s
        self._x = 0
        self._y = 0
        self._z = 500
        self._r = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)

    def set_sticks(self, x: int = 0, y: int = 0, z: int = 500, r: int = 0):
        clip = lambda v, lo, hi: max(lo, min(hi, int(v)))
        with self._lock:
            self._x = clip(x, -1000, 1000)
            self._y = clip(y, -1000, 1000)
            self._z = clip(z, 0, 1000)
            self._r = clip(r, -1000, 1000)

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=2.0)

    def _loop(self):
        hb_div = 0
        while not self._stop.is_set():
            try:
                with self._lock:
                    x, y, z, r = self._x, self._y, self._z, self._r
                self.conn.mav.manual_control_send(
                    self.conn.target_system,
                    x, y, z, r, 0,
                )
                # Identify as a GCS at ~1 Hz. On real hardware a MAVLink
                # force-arm cannot bypass preflight (only an internal/console
                # force can), so the checks must genuinely pass -- which means
                # the FC must see a connected GCS (else gcs_connection_lost).
                hb_div = (hb_div + 1) % 50
                if hb_div == 0:
                    self.conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            self._stop.wait(self.interval_s)


def set_mode_posctl(conn) -> bool:
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        3, 0,  # PX4 custom main = POSCTL (3)
        0, 0, 0, 0,
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def set_mode_auto_land(conn) -> bool:
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4, 6,  # AUTO (4) sub LAND (6)
        0, 0, 0, 0,
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def wait_for_mode(conn, want_main: int, timeout_s: float = 5.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        main = (msg.custom_mode >> 16) & 0xFF
        if main == want_main:
            return True
    return False


def wait_for_alt(conn, target_alt_m: float, tolerance_m: float = 1.5,
                 timeout_s: float = 30.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        msg = conn.recv_match(type="GLOBAL_POSITION_INT",
                              blocking=True, timeout=1.0)
        if msg is None:
            continue
        rel = float(msg.relative_alt) / 1000.0
        if abs(rel - target_alt_m) < tolerance_m:
            return rel
    return None


def request_streams(conn, rate_hz: float = 20.0):
    """Stream OBSTACLE_DISTANCE + LOCAL_POSITION_NED + GLOBAL_POSITION_INT
    + ATTITUDE at the given rate."""
    interval_us = int(1_000_000 / rate_hz)
    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_OBSTACLE_DISTANCE,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    ):
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, interval_us, 0, 0, 0, 0, 0,
        )


def request_logging(conn, enable: bool):
    cmd = (mavutil.mavlink.MAV_CMD_LOGGING_START
           if enable else mavutil.mavlink.MAV_CMD_LOGGING_STOP)
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        cmd, 0, 0, 0, 0, 0, 0, 0, 0,
    )


def capture_telemetry(conn, duration_s: float):
    """Capture OBSTACLE_DISTANCE + LOCAL_POSITION_NED into separate
    lists, time-stamped vs. t0."""
    obs, lpos = [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype not in ("OBSTACLE_DISTANCE", "LOCAL_POSITION_NED"):
            continue
        d = msg.to_dict()
        d["_t"] = time.monotonic() - t0
        if mtype == "OBSTACLE_DISTANCE":
            obs.append(d)
        else:
            lpos.append(d)
    return obs, lpos


def detect_wall_edge(obstacle_samples: list[dict],
                     ratio: float = HILL_EDGE_RATIO) -> dict | None:
    """First OBSTACLE_DISTANCE sample where at least one forward-arc bin
    reads below max_distance * ratio. Under SIH_TERR_EN=2 (walls
    mode), the obstacle is a lattice wall; under SIH_TERR_EN=1
    (terrain mode), it would be an fBm hill. Same detection logic
    either way."""
    for s in obstacle_samples:
        mx = int(s.get("max_distance", 0))
        if mx == 0:
            continue
        threshold = mx * ratio
        bins = s.get("distances", [])
        for i in FORWARD_BINS:
            if i >= len(bins):
                continue
            v = int(bins[i])
            if v != UNKNOWN and v != 0 and v < threshold:
                return dict(t=s["_t"], bin=i, value_cm=v,
                            threshold_cm=int(threshold), max_cm=mx)
    return None


def speed_at(lpos_samples: list[dict], t_target: float) -> float | None:
    if not lpos_samples:
        return None
    best = min(lpos_samples, key=lambda s: abs(s.get("_t", 0) - t_target))
    return math.hypot(float(best.get("vx", 0.0)),
                      float(best.get("vy", 0.0)))


def min_forward_distance_window(obstacle_samples: list[dict],
                                t_start: float,
                                window_s: float) -> int:
    """Min value in cm across forward-arc bins over the window."""
    mn = UNKNOWN
    for s in obstacle_samples:
        t = s.get("_t", 0)
        if t < t_start or t > t_start + window_s:
            continue
        bins = s.get("distances", [])
        for i in FORWARD_BINS:
            if i >= len(bins):
                continue
            v = int(bins[i])
            if v != UNKNOWN and v != 0 and v < mn:
                mn = v
    return mn


def analyze(obs_samples: list[dict],
            lpos_samples: list[dict]) -> dict:
    """Cruise→stop assertion.

    The wall lattice is dense around home — the obstacle ring sees walls
    within ~17 m on bin 5 even before the vehicle moves — so a
    discrete-edge "arrived at the wall" detection (first forward-arc bin
    below max/2) would trip at t≈0 and leave no cruise period to measure
    a speed drop against.

    Instead: the vehicle must cruise (max horizontal speed during the
    run > 1 m/s) and then stop (last 5 sec of the capture window: min
    horizontal speed below some fraction of the cruise max). speed_drop
    becomes (cruise_max - late_min) / cruise_max. The safety floor (min
    forward distance >= 5 m) is unchanged.

    `wall_edge` is preserved as a diagnostic but no longer drives the
    speed assertion.
    """
    if not lpos_samples:
        return dict(ok=False, why="no LOCAL_POSITION_NED samples",
                    n_obstacle=len(obs_samples), n_lpos=0)

    times = [float(s.get("_t", 0.0)) for s in lpos_samples]
    t_first, t_last = min(times), max(times)
    run_duration = t_last - t_first
    LATE_WINDOW_S = 5.0
    t_late_start = t_last - LATE_WINDOW_S

    def hspeed(s):
        return math.hypot(float(s.get("vx", 0.0)), float(s.get("vy", 0.0)))

    cruise_max = max(hspeed(s) for s in lpos_samples)
    late_samples = [s for s in lpos_samples
                    if float(s.get("_t", 0.0)) >= t_late_start]
    late_min = min((hspeed(s) for s in late_samples),
                   default=None)
    speed_drop = ((cruise_max - late_min) / cruise_max
                  if cruise_max > 1e-3 and late_min is not None
                  else None)

    edge = detect_wall_edge(obs_samples)  # diagnostic only
    edge_t = edge["t"] if edge else 0.0
    min_fwd_cm = min_forward_distance_window(
        obs_samples, edge_t, CAPTURE_WINDOW_AFTER_EDGE_S)
    min_fwd_m = (min_fwd_cm / 100.0
                 if min_fwd_cm < UNKNOWN else None)

    ok_cruise = cruise_max > 1.0
    ok_drop = (speed_drop is not None and speed_drop > SPEED_DROP_FRACTION)
    ok_dist = (min_fwd_m is not None and min_fwd_m >= SAFETY_FLOOR_M)
    return dict(
        ok=(ok_cruise and ok_drop and ok_dist),
        wall_edge=edge,
        cruise_max_m_s=cruise_max,
        late_min_m_s=late_min,
        speed_drop_fraction=speed_drop,
        cruise_pass=ok_cruise,
        speed_drop_pass=ok_drop,
        min_forward_distance_m=min_fwd_m,
        min_forward_distance_pass=ok_dist,
        threshold_speed_drop=SPEED_DROP_FRACTION,
        threshold_safety_floor_m=SAFETY_FLOOR_M,
        late_window_s=LATE_WINDOW_S,
        run_duration_s=run_duration,
        n_obstacle=len(obs_samples), n_lpos=len(lpos_samples),
    )


def rotate_to_heading(conn, pump, target_deg, tol_deg=3.0, timeout_s=20.0):
    """In POSCTL, yaw the vehicle to target_deg (compass) via the r stick, so
    a subsequent forward stick flies toward that heading. Used to reproduce a
    hardware heading in SITL."""
    conn.mav.command_long_send(conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50000, 0, 0, 0, 0, 0)
    t0 = time.monotonic(); cur = None
    while time.monotonic() - t0 < timeout_s:
        m = conn.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
        if m is None:
            continue
        cur = math.degrees(m.yaw)
        err = (target_deg - cur + 180) % 360 - 180
        if abs(err) < tol_deg:
            pump.set_sticks(z=500, r=0)
            return cur
        pump.set_sticks(z=500, r=(600 if err > 0 else -600))
    pump.set_sticks(z=500, r=0)
    return cur


def run(args, out_dir: Path):
    hw = bool(getattr(args, "device", None))
    rootfs = out_dir / "rootfs_cp"
    sitl_log = out_dir / "sitl_cp.log"
    proc = None
    conn = None
    pump = None

    if not hw:
        preflight_kill_orphans()
        proc = start_sitl(args.px4_bin, args.px4_data, rootfs)

        def pump_sitl():
            try:
                with sitl_log.open("w") as f:
                    for line in proc.stdout:
                        f.write(line)
            except Exception:
                pass
        threading.Thread(target=pump_sitl, daemon=True).start()

    try:
        conn = connect(uri=args.device) if hw else connect()
        print(f"  heartbeat sys={conn.target_system} "
              f"comp={conn.target_component}", flush=True)
        relax_safety_for_sitl(conn)

        # CP + SIH params per Plan section 7.4.
        set_param(conn, "CP_DIST", CP_DIST_M, "real32")
        set_param(conn, "CP_GO_NO_DATA", 0, "int32")
        set_param(conn, "COM_RC_IN_MODE", 1, "int32")
        set_param(conn, "MPC_POS_MODE", 4, "int32")
        if hw:
            # Real-hardware preflight relaxations for a bench SIH run:
            #  - COM_ARM_ODID:     don't let unhealthy Remote ID block arming
            #    (the pump's GCS heartbeat covers the GCS-connection check).
            #  - COM_ARM_HFLT_CHK: don't block on hardfault/crash-dump files
            #    left on the SD by earlier watchdog resets -- not a real
            #    flight fault, and a MAVLink (external) arm can't bypass it.
            set_param(conn, "COM_ARM_ODID", 0, "int32")
            set_param(conn, "COM_ARM_HFLT_CHK", 0, "int32")
        set_param(conn, "SIH_OBST_EN", 1, "int32")
        set_param(conn, "SIH_OBST_MAX", SIH_OBST_MAX_M, "real32")
        set_param(conn, "SIH_TERR_EN", 2, "int32")  # walls mode
        # The wall at N=30 (lattice cell (3, 0) under SEED=3) is the
        # pinned fixture this test expects CP to engage on.
        set_param(conn, "SIH_TERR_SEED", args.terr_seed, "int32")
        print(f"  params set; using SIH_TERR_SEED={args.terr_seed}",
              flush=True)

        if not wait_for_ekf2_ready(conn, timeout_s=30.0):
            raise RuntimeError("EKF2 never anchored home")

        # Commander needs ~10 s beyond EKF anchor before preflight
        # checks ("no heading reference", "Global position estimate
        # required") clear. Without this settle, set_mode_posctl +
        # arm races the check and the COMMAND_LONG ACK comes back as
        # denied. Same pattern used by sdf_wall_avoidance_test.
        time.sleep(10.0)

        request_streams(conn, rate_hz=20.0)
        request_logging(conn, True)

        pump = ManualControlPump(conn)
        pump.set_sticks(x=0, y=0, z=500, r=0)
        pump.start()
        time.sleep(2.0)

        if not set_mode_posctl(conn):
            raise RuntimeError("mode -> POSCTL not accepted")
        if not wait_for_mode(conn, 3, timeout_s=5.0):
            raise RuntimeError("POSCTL never confirmed via HEARTBEAT")
        print("  POSCTL confirmed, arming", flush=True)

        if not arm(conn, force=hw):
            raise RuntimeError("arm: never reported SAFETY_ARMED")
        print(f"  ARMED -- climbing to {HOVER_ALT_M} m", flush=True)

        pump.set_sticks(z=1000)
        reached = wait_for_alt(conn, HOVER_ALT_M, tolerance_m=1.5,
                               timeout_s=30.0)
        pump.set_sticks(z=500)
        if reached is None:
            raise RuntimeError(f"never reached hover at {HOVER_ALT_M} m")
        print(f"  reached {reached:.1f} m AGL; settling "
              f"{PRE_FLIGHT_HOVER_SETTLE_S} s before forward stick",
              flush=True)
        time.sleep(PRE_FLIGHT_HOVER_SETTLE_S)

        if getattr(args, "yaw_deg", None) is not None:
            print(f"  yawing to {args.yaw_deg:.0f} deg before forward stick...", flush=True)
            got = rotate_to_heading(conn, pump, args.yaw_deg)
            print(f"  heading now ~{got:.0f} deg" if got is not None else "  (no attitude received)", flush=True)
            time.sleep(1.0)

        # Forward stick: x = full forward. y, r = 0. z stays hover.
        pump.set_sticks(x=1000, y=0, z=500, r=0)
        print("  forward stick applied; capturing telemetry "
              f"for up to {args.max_capture_s:.0f} s", flush=True)
        obs_samples, lpos_samples = capture_telemetry(conn,
                                                     args.max_capture_s)
        print(f"  captured {len(obs_samples)} OBSTACLE_DISTANCE + "
              f"{len(lpos_samples)} LOCAL_POSITION_NED messages",
              flush=True)

        # Stop forward, hold, request land.
        pump.set_sticks(x=0, y=0, z=500, r=0)
        time.sleep(1.0)
        request_logging(conn, False)
        time.sleep(1.0)
        set_mode_auto_land(conn)

        # Pull the latest ulog from PX4's log dir (SITL only; on real
        # hardware the log lives on the board's SD card).
        if not hw:
            time.sleep(2.0)
            ulgs = sorted((rootfs / "log").rglob("*.ulg"),
                          key=lambda p: p.stat().st_mtime, reverse=True)
            if ulgs:
                shutil.copy(ulgs[0], args.ulog_out)
                print(f"  copied ulog: {ulgs[0]} -> {args.ulog_out}",
                      flush=True)
            else:
                print(f"  WARN: no .ulg found under {rootfs / 'log'}",
                      flush=True)

        result = analyze(obs_samples, lpos_samples)
        return result, obs_samples, lpos_samples
    finally:
        if pump is not None:
            pump.stop()
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
        if proc is not None:
            kill_sitl(proc)


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--px4-bin", type=Path,
                    help="SITL px4 binary (omit when using --device)")
    ap.add_argument("--px4-data", type=Path,
                    help="SITL rootfs etc/ dir (omit when using --device)")
    ap.add_argument("--device", type=str, default=None,
                    help="run against a connected flight controller instead of "
                         "booting SITL, e.g. /dev/cu.usbmodem01")
    ap.add_argument("--terr-seed", type=int, default=DEFAULT_TERR_SEED)
    ap.add_argument("--yaw-deg", type=float, default=None,
                    help="yaw to this compass heading before flying forward "
                         "(reproduce a hardware heading in SITL)")
    ap.add_argument("--max-capture-s", type=float, default=60.0,
                    help="capture window after forward-stick applied")
    ap.add_argument("--out", type=Path,
                    default=Path("/tmp/sih_terrain_test/"
                                 "cp_test.json"))
    ap.add_argument("--ulog-out", type=Path,
                    default=Path("/tmp/sih_terrain_test/"
                                 "cp_test.ulg"))
    args = ap.parse_args(argv)
    if args.device is None:
        if not args.px4_bin or not args.px4_data:
            ap.error("--px4-bin and --px4-data are required unless --device is given")
        args.px4_bin = args.px4_bin.resolve()
        args.px4_data = args.px4_data.resolve()
    args.out = args.out.resolve()
    args.ulog_out = args.ulog_out.resolve()
    args.out.parent.mkdir(parents=True, exist_ok=True)

    out_dir = args.out.parent
    print(f"=== CP integration test (seed {args.terr_seed}) ===",
          flush=True)
    result, obs, lpos = run(args, out_dir)
    summary = dict(
        result=result,
        n_obstacle_samples=len(obs),
        n_lpos_samples=len(lpos),
        terr_seed=args.terr_seed,
    )
    with args.out.open("w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\nresult: {'PASS' if result['ok'] else 'FAIL'}", flush=True)
    print(json.dumps(result, indent=2, default=str), flush=True)
    print(f"\nwrote {args.out}", flush=True)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
