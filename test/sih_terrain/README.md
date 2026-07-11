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
