#!/usr/bin/env python3
"""Verify SIH samples a real elevation map on hardware (SIH_TERR_EN = 3).

The board loads a .pxtm heightmap from its SD card and every world aware
sensor reads it. This test flies a straight pass across the map in POSCTL
and records two things at once:

  vehicle_local_position_groundtruth   where the vehicle actually was
  distance_sensor                      what the downward beam measured

Ground height under the vehicle is then `truth_alt - measured_range`. That
series is compared against the same .pxtm file sampled on the host at the
same coordinates. A match proves the flight controller is reading the map
correctly, not merely that the file parsed at boot.

Setup, applied automatically:
  SIH_TERR_EN      = 3   map mode
  SIH_DISTSNSR_DIR = 1   downward beam only
  SIH_OBST_EN      = 0   no obstacle ring, this is the rangefinder path
  COM_RC_IN_MODE   = 1   MAVLink MANUAL_CONTROL is the stick source
  SIH_LOC_LAT0/LON0      must already sit inside the map footprint

Usage:
  ./terrain_map_test.py --device /dev/cu.usbmodem01 --map terrain.pxtm
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).parent))
from sitl_harness import (  # noqa: E402
    set_param as h_set_param, wait_for_ekf2_ready, arm as h_arm,
    relax_safety_for_sitl,
)

CRUISE_AGL_M = 120.0         # height above ground to hold during the pass
CLIMB_TIMEOUT_S = 90.0
PASS_DURATION_S = 150.0


# ----------------------------------------------------------------- map


class PxtmMap:
    """Host side reader for the same file the board loaded."""

    def __init__(self, path: Path):
        raw = path.read_bytes()
        if raw[:4] != b"PXTM":
            raise ValueError("not a .pxtm file")
        ver, flags, self.nx, self.ny = struct.unpack_from("<HHHH", raw, 4)
        (self.spacing,) = struct.unpack_from("<f", raw, 12)
        self.lat0, self.lon0 = struct.unpack_from("<dd", raw, 16)
        (self.datum,) = struct.unpack_from("<f", raw, 32)
        n = self.nx * self.ny
        self.samples = struct.unpack_from(f"<{n}h", raw, 64)

    def height(self, north_m: float, east_m: float) -> float:
        """Bilinear with toroidal wrap, matching lib/terrain exactly."""
        fy = north_m / self.spacing
        fx = east_m / self.spacing
        y0f, x0f = math.floor(fy), math.floor(fx)
        ty, tx = fy - y0f, fx - x0f
        y0, x0 = int(y0f) % self.ny, int(x0f) % self.nx
        y1, x1 = (y0 + 1) % self.ny, (x0 + 1) % self.nx
        h00 = self.samples[y0 * self.nx + x0]
        h10 = self.samples[y0 * self.nx + x1]
        h01 = self.samples[y1 * self.nx + x0]
        h11 = self.samples[y1 * self.nx + x1]
        b, c = h10 - h00, h01 - h00
        d = h00 - h10 - h01 + h11
        return h00 + b * tx + c * ty + d * tx * ty


# ------------------------------------------------------------- vehicle


class ManualControlPump:
    """Streams MANUAL_CONTROL at 50 Hz. z = 500 is the POSCTL hold center."""

    def __init__(self, conn, interval_s: float = 0.02):
        self.conn = conn
        self.interval_s = interval_s
        self._x = self._y = self._r = 0
        self._z = 500
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)

    def set_sticks(self, x=0, y=0, z=500, r=0):
        clip = lambda v, lo, hi: max(lo, min(hi, int(v)))
        with self._lock:
            self._x, self._y = clip(x, -1000, 1000), clip(y, -1000, 1000)
            self._z, self._r = clip(z, 0, 1000), clip(r, -1000, 1000)

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=2.0)

    def _loop(self):
        while not self._stop.is_set():
            with self._lock:
                x, y, z, r = self._x, self._y, self._z, self._r
            try:
                self.conn.mav.manual_control_send(
                    self.conn.target_system, x, y, z, r, 0)
            except Exception:
                pass
            time.sleep(self.interval_s)


def set_param(conn, name, value, ptype):
    """MAVLink carries every parameter in a float field. For an INT32 param
    the integer's BIT PATTERN goes in that field, not its numeric value.
    Sending 3.0 for an int param stores 1077936128."""
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        wire = struct.unpack("<f", struct.pack("<i", int(value)))[0]
    else:
        wire = float(value)
    conn.mav.param_set_send(conn.target_system, conn.target_component,
                            name.encode(), wire, ptype)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        r = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if r and r.param_id.strip("\x00") == name:
            return True
    return False


def set_mode_posctl(conn) -> bool:
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3, 0, 0, 0, 0, 0)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def arm(conn) -> bool:
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def disarm(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)


def logging_cmd(conn, start: bool):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_LOGGING_START if start
        else mavutil.mavlink.MAV_CMD_LOGGING_STOP,
        0, 0, 0, 0, 0, 0, 0, 0)


def wait_for_mode(conn, want_main: int, timeout_s: float = 6.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg and ((msg.custom_mode >> 16) & 0xFF) == want_main:
            return True
    return False


def request_streams(conn, rate_hz=25.0):
    for sid in (mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
                mavutil.mavlink.MAV_DATA_STREAM_ALL):
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component, sid, int(rate_hz), 1)


# ---------------------------------------------------------------- main


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/cu.usbmodem01")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--map", type=Path, required=True,
                    help="the same .pxtm that was uploaded to the board")
    ap.add_argument("--out", type=Path, default=Path("terrain_map_test.json"))
    ap.add_argument("--duration", type=float, default=PASS_DURATION_S)
    ap.add_argument("--slow", action="store_true",
                    help="crawl the traverse so the body-fixed beam stays "
                         "near vertical and the range is a true clearance")
    args = ap.parse_args()

    hostmap = PxtmMap(args.map)
    print(f"host map  {hostmap.nx}x{hostmap.ny} @ {hostmap.spacing:.1f} m, "
          f"origin {hostmap.lat0:.6f} {hostmap.lon0:.6f}, datum {hostmap.datum:.0f} m")

    conn = mavutil.mavlink_connection(args.device, baud=args.baud)
    conn.wait_heartbeat(timeout=20)
    print(f"connected sys {conn.target_system}")

    # Failsafes first. A geofence action or the 10 s arm-without-takeoff
    # disarm will quietly end the flight otherwise.
    relax_safety_for_sitl(conn)

    for name, val, kind in (("SIH_TERR_EN", 3, "int32"),
                            ("SIH_DISTSNSR_DIR", 1, "int32"),
                            ("SIH_OBST_EN", 0, "int32"),
                            # CP off. With the ring disabled its no-data branch
                            # zeroes the acceleration setpoint and the vehicle
                            # cannot translate at all.
                            ("CP_DIST", -1.0, "real32"),
                            ("CP_GO_NO_DATA", 1, "int32"),
                            # smooth velocity tracking, as the CP test uses
                            ("MPC_POS_MODE", 4, "int32"),
                            ("COM_ARM_ODID", 0, "int32"),
                            ("COM_ARM_HFLT_CHK", 0, "int32"),
                            ("COM_RC_IN_MODE", 1, "int32"),
                            ("MPC_ALT_MODE", 0, "int32"),
                            ("SIH_DISTSNSR_MAX", 1500.0, "real32"),
                            ("MPC_XY_VEL_MAX", 2.5 if args.slow else 12.0, "real32"),
                            ("MPC_Z_VEL_MAX_UP", 4.0, "real32"),
                            ("MPC_TILTMAX_AIR", 15.0 if args.slow else 45.0, "real32"),
                            ("MPC_ACC_HOR_MAX", 1.0 if args.slow else 5.0, "real32")):
        ok = h_set_param(conn, name, val, kind)
        print(f"  {name:<18} {'ok' if ok else 'FAILED'}")

    request_streams(conn)
    home = wait_for_ekf2_ready(conn, timeout_s=40.0)
    if not home:
        print("EKF2 never anchored a home position"); return 1
    home_lat = home.latitude / 1e7
    home_lon = home.longitude / 1e7
    time.sleep(1.0)

    pump = ManualControlPump(conn)
    pump.set_sticks(z=500)
    pump.start()

    print("logging on")
    logging_cmd(conn, True)
    time.sleep(1.5)

    if not set_mode_posctl(conn):
        print("POSCTL rejected"); pump.stop(); return 1
    if not wait_for_mode(conn, 3, timeout_s=6.0):
        print("POSCTL never confirmed in HEARTBEAT"); pump.stop(); return 1
    print("  POSCTL confirmed")
    # force arm: on hardware the GCS-heartbeat and Remote-ID checks deny a
    # normal arm even when everything relevant is healthy
    if not h_arm(conn, force=True):
        print("arm: never reported SAFETY_ARMED"); pump.stop(); return 1
    print("  ARMED, climbing")

    samples = []
    truth = {"alt": None, "n": None, "e": None}
    rng = {"d": None}

    def collect(deadline):
        while time.monotonic() < deadline:
            msg = conn.recv_match(blocking=True, timeout=0.4)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                truth["n"], truth["e"] = msg.x, msg.y
                truth["alt"] = -msg.z
            elif t == "DISTANCE_SENSOR":
                rng["d"] = msg.current_distance / 100.0
                # one record per beam sample, not per received packet
                if truth["alt"] is not None:
                    samples.append({"t": time.monotonic(), "n": truth["n"],
                                    "e": truth["e"], "alt": truth["alt"],
                                    "range": rng["d"]})

    # climb until the beam reports the target clearance
    pump.set_sticks(z=900)
    t_end = time.monotonic() + CLIMB_TIMEOUT_S
    while time.monotonic() < t_end:
        collect(time.monotonic() + 0.5)
        if rng["d"] is not None and rng["d"] > CRUISE_AGL_M:
            break
    print(f"  climbed to {rng['d']:.1f} m above ground "
          f"({truth['alt']:.1f} m above the origin), starting the pass")
    pump.set_sticks(z=500)
    collect(time.monotonic() + 3.0)

    # translate north east across the map at full stick
    # gentle stick when crawling, so the vehicle never has to pitch far
    pump.set_sticks(x=1000, y=(300 if args.slow else 600), z=500)
    collect(time.monotonic() + args.duration)
    print(f"  captured {len(samples)} samples")

    pump.set_sticks(x=0, y=0, z=500)
    time.sleep(2.0)
    disarm(conn)
    pump.stop()
    time.sleep(1.0)
    logging_cmd(conn, False)
    print("logging off")

    # Where the map's SW corner sits in the vehicle's local NED frame, and how
    # high the map is under home. Mirrors what sih.cpp does at load time.
    d_north = math.radians(hostmap.lat0 - home_lat) * 6371000.0
    d_east = (math.radians(hostmap.lon0 - home_lon) * 6371000.0
              * math.cos(math.radians(home_lat)))
    home_offset = hostmap.height(-d_north, -d_east)
    print(f"  map SW corner at N={d_north:.1f} E={d_east:.1f} m from home; "
          f"map height under home {home_offset:.1f} m")

    # compare the board against the same file, sampled on the host
    errs = []
    for s in samples:
        if s["range"] <= 0.5 or s["range"] > 1490:
            continue
        board_ground = s["alt"] - s["range"]
        # Two corrections, both mandatory, and the test silently passed for the
        # wrong reason without them.
        #
        # 1. `height()` indexes from the map's SW corner, but s["n"]/s["e"] are
        #    NED from home. The firmware places the corner at its true
        #    geographic offset (sih.cpp projects origin_lat/lon, then
        #    set_origin), so the host has to apply the same shift.
        # 2. `height()` returns datum-relative elevation; terrain() is
        #    home-referenced and returns 0 at home by construction. Subtract
        #    the map's value under home to put both in the same frame.
        #
        # They cancel only when home sits on the map's SW corner with the datum
        # equal to home elevation. On any other map the reported error is the
        # map's height at home, not a real disagreement.
        host_ground = hostmap.height(s["n"] - d_north, s["e"] - d_east) - home_offset
        s["board_ground"] = board_ground
        s["host_ground"] = host_ground
        errs.append(abs(board_ground - host_ground))

    args.out.write_text(json.dumps(samples, indent=1))
    print(f"\nwrote {args.out}")
    if errs:
        errs.sort()
        print(f"  usable samples : {len(errs)}")
        print(f"  median error   : {errs[len(errs)//2]:.2f} m")
        print(f"  worst error    : {errs[-1]:.2f} m")
        gr = [s["host_ground"] for s in samples if "host_ground" in s]
        print(f"  ground relief  : {min(gr):.0f} to {max(gr):.0f} m")
        return 0 if errs[len(errs)//2] < 5.0 else 2
    print("  no usable samples")
    return 3


if __name__ == "__main__":
    sys.exit(main())
