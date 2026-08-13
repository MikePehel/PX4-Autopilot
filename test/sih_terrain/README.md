# SIH terrain integration tests

SITL integration tests for the SIH terrain feature. These drive a running
PX4 SITL (`sihsim_quadx`) over MAVLink and assert end-to-end behavior that
the C++ unit tests can't reach. The pure-math side of the feature is covered
by the gtest unit tests in `src/lib/terrain/tests/` and
`src/lib/terrain_sdf/tests/`, which run automatically with `make tests`;
the tests here need a live simulator and are run manually.

`SIH_TERR_EN` is a tri-state ground-mode selector (mutually exclusive):

| Value | Mode | Surface | Walls |
|-------|------|---------|-------|
| 0 | off | flat | none |
| 1 | terrain | fBm heightfield via `SIH_TERR_AMP` / `SIH_TERR_FREQ` | none |
| 2 | walls | flat | procedural lattice |
| 3 | map | real elevation from a `.pxtm` on the SD card | none |

`SIH_TERR_PLANE` (planar slope mode) is honoured independently of
`SIH_TERR_EN`.

## Files

| File | Purpose |
|---|---|
| `sitl_harness.py` | Shared SITL lifecycle helpers (start/kill SITL, connect, set params, wait for EKF2, arm). Imported by the tests; not a test itself. |
| `ground_contact_test.py` | Asserts the vehicle settles at the expected altitude in baseline (`SIH_TERR_EN=0`) and terrain (`SIH_TERR_EN=1`) modes — i.e. ground contact tracks `terrain(N, E)`. Self-contained. |
| `collision_prevention_test.py` | Flies POSCTL into the wall lattice (`SIH_TERR_EN=2`) and asserts CollisionPrevention brakes the vehicle before penetration. |

## Requirements

```bash
pip install pymavlink
```

Build SITL once from the PX4-Autopilot repo root:

```bash
make px4_sitl_default
```

## Run

Each test builds and tears down its own SITL instance:

```bash
python3 test/sih_terrain/ground_contact_test.py \
    --px4-bin  build/px4_sitl_default/bin/px4 \
    --px4-data build/px4_sitl_default/etc

python3 test/sih_terrain/collision_prevention_test.py \
    --px4-bin  build/px4_sitl_default/bin/px4 \
    --px4-data build/px4_sitl_default/etc
```

## Frame convention

Per `src/lib/terrain/terrain.h`: `terrain(N, E)` is in NED with
`terrain(home_N, home_E) = 0`. All assertions use this convention.


## Map mode (`SIH_TERR_EN=3`)

`terrain_map_test.py` and `collision_prevention_test.py --world map` fly
against a real elevation map. Build one with
`Tools/simulation/sih/dem_to_pxtm.py`, copy it to
`/fs/microsd/etc/terrain.pxtm`, and set `SIH_LOC_LAT0/LON0/H0` to a spot
inside its footprint.

**Home has to be chosen, not defaulted.** `lib/terrain` only guarantees a 5 m
flat pad (`FLAT_R_M`); outside it the DEM's real gradient applies, so a sloped
cell makes the four-point landing gear fight the ground on spawn. What a good
home looks like also depends on what is being measured:

| Test | Needs |
|---|---|
| `terrain_map_test.py` | level pad, with relief *anywhere* in reach so the beam has something to measure |
| `collision_prevention_test.py --world map` | level pad, ~60 m of open ground ahead, then terrain rising above cruise altitude inside `SIH_OBST_MAX` |

The second one is easy to get wrong. Terrain that falls away (a rim, a cliff
edge) gives CP nothing to brake for, and terrain that is already close on all
sides makes CP fire before the vehicle can accelerate, so it never reaches
cruise speed and the run fails on `cruise_pass` rather than on braking.

Homes used for the runs recorded in this branch, on a 259x259 @ 30.9 m map of
the Grand Canyon (`--centre 36.078 -112.115 --span-km 8 --spacing 30`):

| Purpose | `SIH_LOC_LAT0` | `SIH_LOC_LON0` | `SIH_LOC_H0` | Why |
|---|---|---|---|---|
| rangefinder accuracy | 36.0640351 | -112.1040995 | 2165.0 | South Rim, level, canyon falls away north giving 1090 m of relief to measure |
| CP vs terrain | 36.1001102 | -112.1040780 | 733.0 | canyon floor, flat for 62 m north then a wall rising +75 m at 93 m and +125 m at 124 m |

Both are also recorded in the flight logs themselves, as the `SIH_LOC_*`
parameters and as `vehicle_local_position.ref_lat/ref_lon/ref_alt`.

### Always reboot after changing `SIH_LOC_*`, and verify it took

SIH reads `SIH_LOC_*` live, but EKF2 anchors its local origin once at startup
and does not re-origin on a parameter change. Change home without a successful
reboot and the two disagree silently: the simulation runs at the new home while
local position is reported against the old one. Nothing errors. Position looks
plausible and is wrong by the distance between the two homes, and every
downstream assertion fails in a way that looks like a code fault.

`wait_for_ekf2_ready()` in `sitl_harness.py` checks for this and refuses to
continue when the origin does not match `SIH_LOC_*`.
