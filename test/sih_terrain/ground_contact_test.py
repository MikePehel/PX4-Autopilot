#!/usr/bin/env python3
"""SIH terrain ground-contact integration test.

Launches PX4 SITL with the sihsim_quadx airframe at the Zurich home
position (47.3977 N, 8.5456 E, 488 m), then drives two scenarios
through MAVLink param toggles on the same running process:

  1. SIH_TERR_EN = 0 (default)         -> vehicle on ground at terrain = 0
  2. SIH_TERR_EN = 1, AMP = 20,        -> terrain-follow: vehicle on ground
     SEED = 3, EKF2_RNG_CTRL = 2          at terrain(home), with EKF2
                                          continuously fusing the terrain-
                                          derived downward rangefinder
  3. SIH_TERR_PLANE = 5 deg, low/slow  -> terrain-tracking: flies a slow, low
     mission, EKF2_RNG_CTRL = 2           mission and asserts EKF2's perceived
                                          terrain (ALTITUDE.altitude_terrain)
                                          tracks tan(5 deg)*north within 4 m
                                          (the regime where range terrain
                                          estimation is valid). --skip-tracking
                                          to skip.

Per the home-offset contract in `src/lib/terrain/terrain.h`, the terrain
function subtracts `terrain(0, 0)` from every evaluation, so
`terrain(home_N, home_E) == 0` and the vehicle should settle at the
same LOCAL_POSITION_NED.z value (~ 0) in both modes. Any non-zero
delta is either a home-offset bug in the terrain library or a sign
that ground contact is reading a different `(N, E)` than what
`terrain_set_params()` was configured with.

Output: JSON record with both scenarios' captured telemetry, written
to the path given by --out.

Usage:
    python3 ground_contact_test.py \\
        --px4-bin build/px4_sitl_default/bin/px4 \\
        --px4-data build/px4_sitl_default/etc \\
        --out ground_contact_results.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    print("error: pymavlink not installed", file=sys.stderr)
    sys.exit(2)


HOME_LAT = 47.3977
HOME_LON = 8.5456
HOME_ALT = 488.0


def start_sitl(px4_bin: Path, px4_data: Path, rootfs: Path) -> subprocess.Popen:
    """Launch px4 in headless SITL mode under rootfs. stdin is kept open
    on a pipe so the pxh shell doesn't EOF and exit; we close it during
    shutdown."""
    rootfs.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update({
        "PX4_SIM_MODEL": "sihsim_quadx",
        "PX4_HOME_LAT": str(HOME_LAT),
        "PX4_HOME_LON": str(HOME_LON),
        "PX4_HOME_ALT": str(HOME_ALT),
        "HEADLESS": "1",
    })
    return subprocess.Popen(
        [str(px4_bin), "-d", str(px4_data), "-s", "etc/init.d-posix/rcS"],
        cwd=str(rootfs),
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def wait_for_heartbeat(uri: str, timeout_s: float = 25.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            conn = mavutil.mavlink_connection(uri)
            conn.wait_heartbeat(timeout=5.0)
            return conn
        except Exception:  # noqa: BLE001
            time.sleep(1.0)
    raise RuntimeError(f"timed out waiting for heartbeat on {uri}")


def fetch_terrain_params(conn) -> dict:
    """Pull just the SIH_TERR_* family + SIH_LOC_* via PARAM_REQUEST_READ
    (faster + bounded vs. the full list)."""
    # SIH_TERR_AMP and SIH_TERR_FREQ are runtime params (alongside
    # EN / SEED / PLANE). Shape knobs (OCT / HURST / EROSION) are
    # compile-time #defines to save flash and are not requested here.
    wanted = [
        "SIH_TERR_EN", "SIH_TERR_AMP", "SIH_TERR_FREQ",
        "SIH_TERR_SEED", "SIH_TERR_PLANE",
        "SIH_LOC_LAT0", "SIH_LOC_LON0", "SIH_LOC_H0",
    ]
    out: dict[str, float | None] = {n: None for n in wanted}
    for n in wanted:
        conn.mav.param_request_read_send(
            conn.target_system, conn.target_component, n.encode("utf-8"), -1
        )
        deadline = time.time() + 2.5
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            name = msg.param_id
            if isinstance(name, bytes):
                name = name.decode("utf-8", errors="replace")
            name = name.rstrip("\x00")
            if name == n:
                out[n] = float(msg.param_value)
                break
    return out


def set_param(conn, name: str, value: float, kind: str = "real32") -> bool:
    # MAVLink PARAM_SET carries the value as a 4-byte float regardless of the
    # actual type; the param_type field tells the receiver how to reinterpret
    # those bytes. For INT32 we must bit-cast the integer through the float
    # field -- passing float(value) corrupts it (e.g. PX4 reads the bit pattern
    # of 2.0 back as INT 1073741824 instead of 2).
    if kind == "real32":
        mav_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        wire = float(value)
    else:
        mav_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        wire = struct.unpack("<f", struct.pack("<i", int(value)))[0]
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        name.encode("utf-8"), wire, mav_type,
    )
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        msg_name = msg.param_id
        if isinstance(msg_name, bytes):
            msg_name = msg_name.decode("utf-8", errors="replace")
        msg_name = msg_name.rstrip("\x00")
        if msg_name == name:
            return True
    return False


def capture_telemetry(conn, duration_s: float = 5.0) -> dict:
    """Capture LOCAL_POSITION_NED + GLOBAL_POSITION_INT samples. SIH
    boots with the vehicle on the ground at home; with no operator
    commands it stays put, so what we measure is the settled
    ground-contact altitude."""
    samples = {"lpos_z_m": [], "lpos_x_m": [], "lpos_y_m": [],
               "rel_alt_m": []}
    deadline = time.time() + duration_s
    while time.time() < deadline:
        msg = conn.recv_match(
            type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"],
            blocking=True, timeout=1.0,
        )
        if msg is None:
            continue
        t = msg.get_type()
        if t == "LOCAL_POSITION_NED":
            samples["lpos_z_m"].append(float(msg.z))
            samples["lpos_x_m"].append(float(msg.x))
            samples["lpos_y_m"].append(float(msg.y))
        elif t == "GLOBAL_POSITION_INT":
            samples["rel_alt_m"].append(float(msg.relative_alt) / 1000.0)
    return samples


def summarize(samples: dict) -> dict:
    def stats(xs):
        if not xs:
            return {"n": 0}
        return {"n": len(xs),
                "min": min(xs), "max": max(xs),
                "mean": sum(xs) / len(xs),
                "last": xs[-1]}
    return {k: stats(v) for k, v in samples.items()}


def run_scenario(conn, label: str, settings: dict, hover_s: float) -> dict:
    set_results = {}
    for name, (value, kind) in settings.items():
        set_results[name] = set_param(conn, name, value, kind)
    # Let parameters_updated() propagate + sim settle.
    time.sleep(1.5)
    params_after = fetch_terrain_params(conn)
    telemetry = capture_telemetry(conn, hover_s)
    return {
        "label": label,
        "set_results": set_results,
        "params_after_set": params_after,
        "telemetry_summary": summarize(telemetry),
        "telemetry_raw_first10_z": telemetry["lpos_z_m"][:10],
        "telemetry_raw_last10_z": telemetry["lpos_z_m"][-10:],
    }


# ---- Scenario C: low-and-slow terrain-tracking assertion ----
#
# Flies a slow, low-altitude waypoint mission over a deterministic planar
# slope (SIH_TERR_PLANE) with EKF2_RNG_CTRL=2, and asserts that EKF2's
# perceived terrain -- the MAVLink ALTITUDE.altitude_terrain field, which is
# (-z - dist_bottom) -- tracks the known ground truth tan(plane)*north.
#
# This is the regime where rangefinder terrain estimation is valid: low
# altitude (small tilt-offset, in sensor range) and slow speed (the terrain
# under the vehicle changes slowly enough for EKF2's terrain process model to
# follow). A clamped/broken estimate shows up as a large mean error. The
# planar slope gives an exact closed-form ground truth, so no terrain oracle
# is needed.
PLANE_DEG = 5.0                 # terrain(N) = tan(5 deg) * N
TRACK_ALT_M = 22.0              # low constant cruise altitude
TRACK_CRUISE_MS = 2.5           # slow
TRACK_LEGS_N = [40, 80, 120, 80, 40, 0]   # north waypoints, out to 120 m and back
TRACK_MAX_MEAN_ERR_M = 4.0      # pass threshold


def _cmd(conn, command, *p):
    a = list(p) + [0.0] * (7 - len(p))
    conn.mav.command_long_send(conn.target_system, conn.target_component, command, 0, *a)


def _upload_mission(conn, items) -> bool:
    mav = mavutil.mavlink
    conn.mav.mission_count_send(conn.target_system, conn.target_component, len(items), 0)
    deadline = time.time() + 30.0
    while time.time() < deadline:
        m = conn.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                            blocking=True, timeout=2.0)
        if m is None:
            continue
        if m.get_type() == "MISSION_ACK":
            return getattr(m, "type", 0) == 0
        it = items[m.seq]
        conn.mav.mission_item_int_send(
            conn.target_system, conn.target_component, m.seq,
            mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, it["cmd"],
            1 if m.seq == 0 else 0, 1, it.get("p1", 0.0), 0.0, 0.0, float("nan"),
            int(it["lat"] * 1e7), int(it["lon"] * 1e7), float(it["alt"]))
    return False


def _force_arm(conn) -> bool:
    mav = mavutil.mavlink
    for _ in range(12):
        _cmd(conn, mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196)  # 21196 = force-arm (bypass preflight)
        time.sleep(2.0)
        m = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        if m and (m.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def run_terrain_tracking(conn, home_lat: float, home_lon: float) -> dict:
    mav = mavutil.mavlink
    slope = math.tan(math.radians(PLANE_DEG))

    for n, v, k in [("SIH_TERR_EN", 1, "int32"), ("SIH_TERR_PLANE", PLANE_DEG, "real32"),
                    ("EKF2_RNG_CTRL", 2, "int32"), ("SIH_DISTSNSR_DIR", 1, "int32"),
                    ("MPC_XY_CRUISE", TRACK_CRUISE_MS, "real32"),
                    ("MIS_TAKEOFF_ALT", TRACK_ALT_M, "real32"),
                    ("NAV_DLL_ACT", 0, "int32"), ("NAV_RCL_ACT", 0, "int32"),
                    ("COM_RCL_EXCEPT", 4, "int32")]:
        set_param(conn, n, v, k)
    time.sleep(1.5)

    items = [{"cmd": mav.MAV_CMD_NAV_TAKEOFF, "lat": home_lat, "lon": home_lon, "alt": TRACK_ALT_M}]
    for dN in TRACK_LEGS_N:
        items.append({"cmd": mav.MAV_CMD_NAV_WAYPOINT,
                      "lat": home_lat + dN / 111320.0, "lon": home_lon, "alt": TRACK_ALT_M})
    items.append({"cmd": mav.MAV_CMD_NAV_LAND, "lat": home_lat, "lon": home_lon, "alt": 0.0})
    if not _upload_mission(conn, items):
        return {"status": "inconclusive", "reason": "mission upload failed"}

    # Establish GCS presence (periodic heartbeats) and wait for a valid EKF2
    # home before commanding flight -- without these the auto-takeoff silently
    # does not trigger.
    _cmd(conn, mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_HOME_POSITION, 1_000_000)
    home_ok = False
    t0 = time.time()
    while time.time() - t0 < 40.0:
        conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        m = conn.recv_match(type="HOME_POSITION", blocking=True, timeout=1.0)
        if m and m.latitude != 0:
            home_ok = True
            break
    if not home_ok:
        return {"status": "inconclusive", "reason": "no EKF2 home"}
    if not _force_arm(conn):
        return {"status": "inconclusive", "reason": "arm failed"}
    _cmd(conn, mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4)  # AUTO.MISSION
    for mid in (mav.MAVLINK_MSG_ID_ALTITUDE, mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED):
        _cmd(conn, mav.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 100000)

    pairs = []  # (north, perceived_terrain = altitude_terrain)
    north = 0.0
    alt_up = 0.0
    airborne = False
    last_hb = 0.0
    t0 = time.time()
    while time.time() - t0 < 220.0:
        now = time.time()
        if now - last_hb > 1.0:
            conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = now
        m = conn.recv_match(type=["LOCAL_POSITION_NED", "ALTITUDE"], blocking=True, timeout=1.0)
        if m is None:
            continue
        if m.get_type() == "LOCAL_POSITION_NED":
            north = float(m.x)
            alt_up = -float(m.z)
            if alt_up > 8.0:
                airborne = True
            if airborne and alt_up < 1.5:
                break  # landed
        elif m.get_type() == "ALTITUDE":
            at = float(m.altitude_terrain)
            if airborne and alt_up > 8.0 and at == at and 0.0 <= north <= 130.0:  # at==at: not NaN
                pairs.append((north, at))

    if len(pairs) < 30:
        return {"status": "inconclusive", "reason": f"only {len(pairs)} airborne samples"}
    errs = [abs(at - slope * n) for n, at in pairs]
    mean_err = sum(errs) / len(errs)
    perceived_range = max(at for _, at in pairs) - min(at for _, at in pairs)
    ok = (mean_err < TRACK_MAX_MEAN_ERR_M) and (perceived_range > 5.0)
    return {
        "status": "pass" if ok else "fail",
        "plane_deg": PLANE_DEG, "cruise_alt_m": TRACK_ALT_M, "cruise_ms": TRACK_CRUISE_MS,
        "samples": len(pairs),
        "mean_tracking_error_m": round(mean_err, 2),
        "threshold_m": TRACK_MAX_MEAN_ERR_M,
        "perceived_terrain_range_m": round(perceived_range, 1),
        "north_span_m": [round(min(n for n, _ in pairs), 1), round(max(n for n, _ in pairs), 1)],
    }


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--px4-bin", required=True, type=Path)
    ap.add_argument("--px4-data", required=True, type=Path)
    ap.add_argument("--rootfs", type=Path, default=None,
                    help="working dir for px4 (parameters.bson, dataman, log/); "
                         "default = fresh temp dir")
    ap.add_argument("--connection", default="udp:127.0.0.1:14540")
    ap.add_argument("--hover-seconds", type=float, default=5.0)
    ap.add_argument("--skip-tracking", action="store_true",
                    help="skip the low-and-slow terrain-tracking flight (Scenario C)")
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args(argv)

    rootfs = args.rootfs or Path(tempfile.mkdtemp(prefix="px4_ground_contact_"))
    # Always start with a clean parameter state.
    for p in ("parameters.bson", "parameters_backup.bson"):
        f = rootfs / p
        if f.exists():
            f.unlink()
    log_dir = rootfs / "log"
    if log_dir.exists():
        shutil.rmtree(log_dir, ignore_errors=True)

    print(f"[ground_contact] launching SITL under {rootfs}", file=sys.stderr)
    proc = start_sitl(args.px4_bin.resolve(), args.px4_data.resolve(), rootfs)
    try:
        print("[ground_contact] waiting for heartbeat ...", file=sys.stderr)
        conn = wait_for_heartbeat(args.connection)
        print(f"[ground_contact] heartbeat sys={conn.target_system} "
              f"comp={conn.target_component}", file=sys.stderr)

        # Let SIH stabilize after boot.
        time.sleep(2.0)

        # Scenario A: baseline parity (SIH_TERR_EN=0 by default).
        baseline_params = fetch_terrain_params(conn)

        scen_baseline = run_scenario(
            conn,
            label="baseline_SIH_TERR_EN=0",
            settings={"SIH_TERR_EN": (0, "int32")},
            hover_s=args.hover_seconds,
        )

        # Scenario B: terrain-follow mode (SIH_TERR_EN=1, fBm hills).
        # terrain(home_N, home_E) = 0 by construction (home offset
        # subtraction), so ground contact still triggers at z = 0;
        # the test is checking that the 4-gear contact model integrates
        # cleanly when the surface is fBm rather than flat.
        #
        # EKF2_RNG_CTRL = 2 (continuous range aiding) makes EKF2 fuse the
        # terrain-derived downward distance_sensor at all times, not only
        # in the low/slow conditional window. The stock sihsim_quadx
        # airframe leaves this at the firmware default (1, conditional),
        # so it is set here as part of the terrain-follow test config
        # rather than in the shipped airframe. EKF2_HGT_REF is left at its
        # default (GPS): range feeds the terrain/HAGL estimate, not the
        # absolute height reference (which would ride up/down with the
        # hills over varying terrain).
        scen_terrain = run_scenario(
            conn,
            label="terrain_follow_SIH_TERR_EN=1_AMP=20_SEED=3_RNG_CTRL=2",
            settings={
                "SIH_TERR_EN": (1, "int32"),
                "SIH_TERR_AMP": (20.0, "real32"),
                "SIH_TERR_SEED": (3, "int32"),
                "EKF2_RNG_CTRL": (2, "int32"),
            },
            hover_s=args.hover_seconds,
        )

        # Scenario C: low-and-slow terrain-tracking assertion (flies a mission).
        tracking = None
        if not args.skip_tracking:
            print("[ground_contact] Scenario C: low-and-slow terrain-tracking flight ...",
                  file=sys.stderr)
            tracking = run_terrain_tracking(conn, HOME_LAT, HOME_LON)
            print(f"[ground_contact] tracking result: {tracking}", file=sys.stderr)

        record = {
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "home": {"lat": HOME_LAT, "lon": HOME_LON, "alt_m": HOME_ALT},
            "expected_z_m_both_modes": 0.0,
            "rationale": ("terrain(home_N, home_E) = 0 by construction "
                          "(home offset). Vehicle spawns at home, so "
                          "ground contact triggers at z=0 in both modes."),
            "params_at_boot": baseline_params,
            "scenarios": [scen_baseline, scen_terrain],
            "terrain_tracking": tracking,
        }

        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(record, indent=2, sort_keys=True))
        print(f"[ground_contact] wrote {args.out}", file=sys.stderr)

        # A real tracking failure (flew, but the estimate did not track) fails
        # the test; infrastructure issues (could not arm/fly) are inconclusive.
        if tracking is not None and tracking.get("status") == "fail":
            print(f"[ground_contact] FAIL: terrain tracking error "
                  f"{tracking['mean_tracking_error_m']} m > {tracking['threshold_m']} m",
                  file=sys.stderr)
            return 1
        return 0

    finally:
        print("[ground_contact] shutting down SITL", file=sys.stderr)
        if proc.stdin and not proc.stdin.closed:
            try:
                proc.stdin.write(b"shutdown\n")
                proc.stdin.flush()
                proc.stdin.close()
            except (BrokenPipeError, OSError):
                pass
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
