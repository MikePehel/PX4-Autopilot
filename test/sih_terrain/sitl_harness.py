#!/usr/bin/env python3
"""Shared SITL lifecycle helpers for the SIH terrain integration tests.

These functions handle the boilerplate of driving a PX4 SITL instance over
MAVLink: spawning the `sihsim_quadx` airframe, killing it (and any orphans)
cleanly between runs, connecting via pymavlink, setting parameters, waiting
for the estimator to anchor, and arming. The SIH terrain integration tests
import from here instead of each re-implementing the same lifecycle.

Requires pymavlink (`pip install pymavlink`).
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import math
import time
from pathlib import Path

# OBSTACLE_DISTANCE is msg ID 330 -- only present in MAVLink v2 (IDs > 255).
# Set this before importing pymavlink so its generated headers expose the
# message constants the tests need.
os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not installed. Run: pip install pymavlink", file=sys.stderr)
    sys.exit(2)


# ---------- SITL lifecycle ---------------------------------------------------

def start_sitl(px4_bin: Path, px4_data: Path, rootfs: Path) -> subprocess.Popen:
    """Spawn PX4 SITL with the sihsim_quadx airframe. Returns the Popen handle."""
    env = os.environ.copy()
    env["PX4_SIM_MODEL"] = "sihsim_quadx"
    env["PX4_SIMULATOR"] = "sihsim"
    rootfs.mkdir(parents=True, exist_ok=True)
    # PX4 boots from cwd and resolves init.d-posix/airframes via cwd/etc/.
    # Symlink rootfs/etc -> the build's etc dir so airframe scripts are reachable.
    etc_link = rootfs / "etc"
    if etc_link.is_symlink() or etc_link.exists():
        etc_link.unlink()
    etc_link.symlink_to(px4_data)
    proc = subprocess.Popen(
        [str(px4_bin), "-d", "-s", str(px4_data / "init.d-posix/rcS"),
         str(rootfs)],
        env=env, cwd=str(rootfs),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
        preexec_fn=os.setsid,  # new process group for clean group-kill later
    )
    # let it boot (heartbeat typically visible within ~10s after sensors come up)
    time.sleep(10)
    return proc


def kill_sitl(proc: subprocess.Popen):
    """Kill the whole PX4 process group + wait for OS to release the UDP port."""
    if proc.poll() is None:
        pgid = None
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            pass
        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGTERM)
                proc.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(pgid, signal.SIGKILL)
                    proc.wait(timeout=3)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    pass
    # Belt-and-suspenders: also pkill any orphan px4 from prior failed runs
    subprocess.run(["pkill", "-9", "-f", "/bin/px4"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # OS needs a moment to release the UDP port for the next scenario
    time.sleep(6)


def preflight_kill_orphans():
    """Before any scenario, make sure no px4 from a prior aborted run is around."""
    subprocess.run(["pkill", "-9", "-f", "/bin/px4"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)


# ---------- MAVLink connect / params -----------------------------------------

def connect(uri: str = "udp:127.0.0.1:14540", timeout_s: float = 25.0):
    conn = mavutil.mavlink_connection(uri, source_system=255)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if conn.wait_heartbeat(timeout=1):
            return conn
    raise TimeoutError(f"no heartbeat on {uri} within {timeout_s}s")


def set_param(conn, name: str, value, kind: str = "real32") -> bool:
    """PARAM_SET with ack wait. kind in {real32, int32}.

    MAVLink PARAM_SET carries the value as a 4-byte float on the wire
    regardless of the actual type; the param_type field tells the
    receiver how to reinterpret those four bytes. For int32 we have to
    bit-cast the integer through the float field -- using float(int)
    converts the value semantically and the receiver then reads the
    (e.g.) bit pattern 0x40800000 as INT 1082130432 instead of 4.
    """
    import struct
    if kind == "real32":
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        val = float(value)
    else:
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        # Pack the int as 4 bytes, reinterpret those bytes as a float so
        # the receiver's INT32 reinterpret reads back the original int.
        val = struct.unpack("<f", struct.pack("<i", int(value)))[0]
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        name.encode("ascii"), val, ptype,
    )
    # wait up to 2s for echo
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.strip("\x00") == name:
            return msg
    return None


# ---------- flight prep ------------------------------------------------------

def wait_for_ekf2_ready(conn, timeout_s: float = 30.0):
    """Block until EKF2 publishes a valid global origin (HOME_POSITION).

    Returns the HOME_POSITION message, or None on timeout. Callers that only
    care whether it worked can keep testing it as a boolean; callers that need
    to place a map against home need the lat/lon, and re-requesting it after
    the fact races with the vehicle arming.

    Without this, mode switches to AUTO.MISSION silently fail because PX4
    refuses to accept a global mission when the estimator hasn't anchored
    its origin yet. In SITL with SIH this typically takes ~8-15 s after boot.
    """
    # Ask for HOME_POSITION at 1 Hz so it lands in the stream
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
        1_000_000, 0, 0, 0, 0, 0,
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        msg = conn.recv_match(type="HOME_POSITION", blocking=True, timeout=1.0)
        if msg and msg.latitude != 0 and msg.longitude != 0:
            print(f"  EKF2 ready: home lat={msg.latitude/1e7:.6f} "
                  f"lon={msg.longitude/1e7:.6f} alt={msg.altitude/1000:.1f}m",
                  flush=True)

            if not _origin_matches_sih(conn, msg):
                return None

            return msg
    return None


def _origin_matches_sih(conn, home, tol_m: float = 50.0) -> bool:
    """Refuse to fly when EKF2's origin and SIH's home have drifted apart.

    SIH reads SIH_LOC_* live, but EKF2 anchors its local origin once at
    startup and does not re-origin on a parameter change. Change home without
    a successful reboot and the two disagree silently: the simulation runs at
    the new home while local position is reported against the old one. Nothing
    errors, position looks plausible, and every assertion downstream fails in a
    way that looks like a code fault rather than a stale origin.

    Cheap to check, and it has cost entire afternoons when it was not.
    """
    want = {}
    for name in ("SIH_LOC_LAT0", "SIH_LOC_LON0", "SIH_LOC_H0"):
        conn.mav.param_request_read_send(
            conn.target_system, conn.target_component, name.encode(), -1)

    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0 and len(want) < 3:
        pv = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if pv and pv.param_id in ("SIH_LOC_LAT0", "SIH_LOC_LON0", "SIH_LOC_H0"):
            want[pv.param_id] = pv.param_value

    if len(want) < 3:
        print("  WARNING: could not read SIH_LOC_*; origin not verified", flush=True)
        return True

    lat, lon, alt = home.latitude / 1e7, home.longitude / 1e7, home.altitude / 1000.0
    d_north = (lat - want["SIH_LOC_LAT0"]) * 110574.0
    d_east = ((lon - want["SIH_LOC_LON0"]) * 111320.0
              * math.cos(math.radians(lat)))
    d_up = alt - want["SIH_LOC_H0"]

    if max(abs(d_north), abs(d_east), abs(d_up)) <= tol_m:
        return True

    print(f"  EKF2 origin does not match SIH_LOC_*: "
          f"N{d_north:+.0f} E{d_east:+.0f} U{d_up:+.0f} m", flush=True)
    print("  The board did not reboot after SIH_LOC_* changed. Reboot and "
          "confirm the USB device re-enumerates before flying.", flush=True)
    return False


def arm(conn, force: bool = False) -> bool:
    """COMMAND_LONG arm + wait for SAFETY_ARMED in HEARTBEAT.

    force=True sets param2=21196 to bypass preflight -- needed on real
    hardware, where GCS-heartbeat / Remote-ID checks (absent in SITL) deny a
    normal arm."""
    p2 = 21196 if force else 0
    t0 = time.monotonic()
    last_send = -1.0
    while time.monotonic() - t0 < 12.0:
        # Resend ~1 Hz: on hardware preflight may take a second or two to
        # clear (e.g. once the GCS heartbeat registers), so a single shot
        # can race the checks.
        if time.monotonic() - last_send > 1.0:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, p2, 0, 0, 0, 0, 0,
            )
            last_send = time.monotonic()
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return msg
    return None


def relax_safety_for_sitl(conn):
    """Disable failsafes that would block or kill a scripted SITL mission."""
    # Data-link loss action: disable (0 = disabled)
    set_param(conn, "NAV_DLL_ACT", 0, "int32")
    # RC loss action: disable
    set_param(conn, "NAV_RCL_ACT", 0, "int32")
    # Geofence action: none
    set_param(conn, "GF_ACTION", 0, "int32")
    # Auto-disarm-after-arm-without-takeoff: extend from 10s to 60s
    set_param(conn, "COM_DISARM_PRFLT", 60.0, "real32")
    # Lower preflight thresholds for SITL convenience
    set_param(conn, "COM_ARM_WO_GPS", 0, "int32")  # require GPS
