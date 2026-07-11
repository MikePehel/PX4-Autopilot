/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file terrain.h
 *
 * Procedural terrain library for SIH (Simulation-In-Hardware).
 *
 * Provides a pure deterministic terrain height function and a beam-vs-terrain
 * raycast helper. Used by the SIH simulator to give the otherwise-flat
 * `z = 0` floor a procedural shape and to feed world-aware sensors
 * (downward / forward / side rangefinders) with realistic ground returns.
 *
 * ## Frame convention (pinned)
 *
 * - `terrain(north_m, east_m)` operates in NED (North-East-Down). North
 *   increases northward, east eastward, both in metres from the home
 *   reference set via `terrain_set_params()`.
 * - By construction `terrain(home_N, home_E) = 0`. This is enforced by
 *   subtracting `terrain(0, 0)` from every evaluation so the vehicle
 *   never spawns buried or floating at `SIH_LOC_H0`.
 * - Height is positive *up*. Returning `+5.f` means the terrain surface
 *   sits 5 m above the home altitude reference.
 * - The companion viewer uses raylib Y-up. It is responsible for applying
 *   the NED -> Y-up transform `(N, E, D) -> (E, -D, N)` *before* evaluating
 *   this function on its side. The function itself knows nothing about Y-up.
 *
 * ## Shared source contract
 *
 * This file (and `terrain.c`) is the physical source of truth for the
 * terrain shape on BOTH the firmware side (PX4) and the companion viewer
 * side. Both compile the same bytes. Constraints:
 *
 * - Pure C (no C++, no STL, no PX4 headers).
 * - Only `<math.h>` essentials (`sinf`, `cosf`, `floorf`, `sqrtf`, `powf`).
 * - No global heap, no dynamic allocation.
 * - The integer hash layer is bit-exact across ARM Cortex-M7 and x86_64.
 *   Float fBm accumulation may drift by ~1 ULP between architectures;
 *   this is invisible visually but matters for cross-platform numerical
 *   diff testing.
 *
 * ## API stability
 *
 * The signatures below are stable. Adding new functions is fine; changing
 * existing ones requires explicit coordination so the companion viewer
 * stays in sync.
 *
 * For arbitrary-direction raycasts against obstacles (forward/side/up
 * beams, the obstacle ring), see `lib/terrain_sdf`; this library handles
 * only the heightfield and the near-vertical analytical fast-path.
 */

#ifndef PX4_SRC_LIB_TERRAIN_TERRAIN_H_
#define PX4_SRC_LIB_TERRAIN_TERRAIN_H_

/**
 * Compile-time radius of the flat takeoff zone around home. The fBm
 * value and gradient are forced to 0 inside this disk by a hard cutoff
 * in `terrain()` / `terrain_gradient()`. There is no smooth ramp —
 * the surface steps from 0 to whatever the noise gives at `r =
 * FLAT_R_M`. For vertical takeoff/landing inside the disk this
 * discontinuity is invisible (vehicle never crosses the boundary in
 * contact); for vehicles taxiing across the boundary the
 * spring-damper landing gear absorbs the step within its compliance
 * range.
 *
 * Set to 0.f to disable (terrain reverts to pure fBm with no flat
 * zone). The default of 5 m gives the SITL acceptance tests a
 * deterministic flat pad. The companion viewer reads the same #define;
 * both sides agree by construction without any param wire.
 */
#define FLAT_R_M  5.0f

/**
 * Evaluate terrain height at a horizontal (north, east) location.
 *
 * Computes multi-octave value noise with octave rotation and a
 * slope-attenuated divisor (Quilez 2009 fBm). The home reference offset
 * is applied so `terrain(home_N, home_E) == 0` to float precision.
 *
 * Flat takeoff zone: when `FLAT_R_M > 0`,
 * `terrain()` returns exactly 0 for any (N, E) inside the disk of
 * radius `FLAT_R_M` around home. Outside the disk the procedural fBm
 * noise resumes at full amplitude — there is no smooth ramp, the
 * surface has a hard step at the boundary.
 *
 * Cost: ~6 us per call on FMUv6 with 6 octaves (the default). Linear
 * in octave count.
 *
 * @param north_m  North coordinate in metres from the home reference.
 * @param east_m   East coordinate in metres from the home reference.
 * @return         Terrain height in metres (positive up).
 */
float terrain(float north_m, float east_m);

/**
 * Evaluate the analytic gradient of `terrain()` at a location.
 *
 * The slope-attenuated fBm accumulator computes the running gradient
 * `(d h / d N, d h / d E)` as part of evaluating the divisor term, so
 * this function is essentially free alongside a `terrain()` call. The
 * gradient is used by the sloped-contact landing-gear model and by
 * the rangefinder quality model to detect steep oblique returns.
 *
 * Inside the flat takeoff zone (`r < FLAT_R_M`) the gradient is
 * `(0, 0)` by the same hard cutoff that zeroes `terrain()`.
 *
 * @param north_m  North coordinate in metres from the home reference.
 * @param east_m   East coordinate in metres from the home reference.
 * @param dnorth   Out: partial derivative of height w.r.t. north (unitless).
 * @param deast    Out: partial derivative of height w.r.t. east (unitless).
 */
void terrain_gradient(float north_m, float east_m, float *dnorth, float *deast);

/**
 * Cast a near-vertical ray against the procedural terrain and return
 * the distance along the beam to the first intersection.
 *
 * ANALYTICAL FAST-PATH ONLY. Use this helper for beams whose tilt off
 * the local vertical is small (≤ ~5°) — typically the downward
 * rangefinder on a level vehicle. For arbitrary-direction beams
 * (forward / side / up sensors, the obstacle ring), route through
 * `sdf_sphere_trace` in `lib/terrain_sdf` instead, which handles
 * steep oblique and upward beams correctly against both the
 * heightfield and a primitive scene.
 *
 * The beam starts at `(origin_n, origin_e, origin_alt)` with the
 * altitude axis positive UP (i.e. the world-frame altitude that
 * `terrain()` returns), and travels in direction
 * `(dir_n, dir_e, dir_alt)` where `dir_alt` is positive when the ray
 * points UP — a straight-down beam has `dir_alt = -1.f`. The direction
 * vector should be unit length; non-unit directions still produce a
 * correct intersection but the returned `t` then scales with the
 * input magnitude.
 *
 * For a near-vertical downward beam, the closed-form intersection is
 *
 *   t = (origin_alt - terrain(origin_n, origin_e)) / (-dir_alt)
 *
 * (a downward unit beam has `-dir_alt = 1`, so for pure-down `t` is
 * simply the altitude above ground). The analytical path approximates
 * the terrain function as locally flat over the small horizontal
 * projection a ≤5° beam covers — exact for straight-down
 * (`dir_n = dir_e = 0`), and sub-sensor-noise at the tilts a downward
 * rangefinder typically sees.
 *
 * If the input tilt is too large for the analytical approximation
 * (i.e. the beam is more than ~5° off vertical, including any upward
 * beam), this helper returns `max_t` (miss) rather than computing an
 * inaccurate answer. Callers must detect that case and re-issue via
 * `sdf_sphere_trace`.
 *
 * Returns `max_t` when no intersection is found within the search
 * range (e.g. an upward beam over open terrain) OR when the beam is
 * outside the analytical-fast-path envelope. The caller distinguishes
 * "hit" from "miss / out-of-envelope" by comparing the returned value
 * to `max_t`.
 *
 * @param origin_n   Ray origin north [m].
 * @param origin_e   Ray origin east [m].
 * @param origin_alt Ray origin altitude [m] (positive up).
 * @param dir_n      Ray direction north component (unit-length expected).
 * @param dir_e      Ray direction east component.
 * @param dir_alt    Ray direction altitude component (positive = pointing
 *                   UP; `-1.f` for a straight-down beam).
 * @param max_t      Maximum search distance along the ray [m].
 * @return           Distance along the ray to the first terrain hit, or
 *                   `max_t` if no hit was found within range or the
 *                   beam is outside the analytical envelope.
 */
float raycast(float origin_n, float origin_e, float origin_alt,
	      float dir_n, float dir_e, float dir_alt, float max_t);

/**
 * Configure the terrain function's parameters.
 *
 * Called by SIH at startup and on `parameters_updated()` to push the
 * `SIH_TERR_*` parameter values into the static state used by `terrain()`
 * and `raycast()`. This is the only piece of mutable state in the
 * library, and it is set-once-per-config-change rather than per-evaluation.
 *
 * When `amp == 0.f`, the terrain function returns `0.f` everywhere
 * (regardless of other parameters). This is the byte-identical-baseline
 * mode used when `SIH_TERR_EN = 0`: SIH calls
 * `terrain_set_params(0.f, ...)` and every subsequent `terrain()` call
 * is a free zero.
 *
 * When `plane_deg != 0.f`, the library enters deterministic planar mode:
 * `terrain(N, E) = tan(plane_deg) * N` and `terrain_gradient` returns
 * `(tan(plane_deg), 0)`. fBm evaluation is short-circuited entirely; the
 * `amp` / `wavelength` / `oct` / `hurst` / `erosion` / `seed` arguments
 * are stored but unused on the hot path. `terrain(0, 0) == 0` is
 * preserved by construction (no home-offset machinery needed). The mode
 * exists so acceptance tests that need a slope that is uniform across a
 * gear-footprint patch (e.g. the multirotor slope-tip test under
 * `sloped_landing_test.py`) have a deterministic surface to land on,
 * which fBm at the lattice origin cannot provide. When `plane_deg == 0.f`
 * the function is byte-identical to its pre-planar behaviour.
 *
 * The `home_n` / `home_e` arguments pin the home reference so that
 * `terrain(home_n, home_e) == 0` after this call. The current
 * implementation stores `(home_n, home_e)` and subtracts the noise value
 * at that point from every evaluation; the cost is one extra
 * subtraction per call. In planar mode these are stored but unused.
 *
 * @param amp         Peak amplitude of terrain variation [m]. 0 = flat.
 * @param wavelength  Base wavelength of the largest octave [m]. Smaller
 *                    values produce finer detail. The corresponding
 *                    SIH parameter is `SIH_TERR_FREQ` (1/m); call sites
 *                    invert: `wavelength = 1.f / max(freq, 1e-6f)`.
 * @param oct         Number of fBm octaves to accumulate [1..9].
 * @param hurst       Hurst exponent [0.3..1.0]. Higher = smoother.
 * @param erosion     Slope-attenuated divisor strength [0..5].
 * @param seed        Integer hash seed for reproducible worlds.
 * @param home_n      North coordinate of home reference [m].
 * @param home_e      East coordinate of home reference [m].
 * @param plane_deg   Planar-mode slope angle [deg], tilted toward +N.
 *                    0 = disabled (fBm mode). Non-zero engages the
 *                    deterministic planar half-space described above.
 *                    Valid range matches the `SIH_TERR_PLANE` param:
 *                    (-89, 89) to avoid vertical-plane degeneracy.
 */
void terrain_set_params(float amp, float wavelength, int seed,
			float home_n, float home_e,
			float plane_deg);

/**
 * Return the size in bytes of any lookup table the terrain library holds.
 *
 * The current implementation is hash-based and uses zero lookup-table
 * memory; this accessor returns 0. Provided as a discoverable answer to
 * "how much RAM does terrain.c cost?" without anyone having to read the
 * source.
 */
unsigned int terrain_seed_lookup_table_size(void);

/**
 * Accessor for the integer hash seed currently configured via
 * `terrain_set_params()`. Exposed so SDF scene generators in
 * `lib/terrain_sdf` can derive primitive positions from the same shared
 * seed without re-plumbing it through the param chain.
 */
int terrain_get_seed(void);

/*============================================================================
 * Integer hash primitives — public API for SDF scene generators.
 *
 * Bit-exact contract: uint32 multiplies and xor-shifts only, well-defined
 * across ARM Cortex-M7, x86_64, and wasm32. Both PX4 firmware and the
 * companion viewer compile the same source and produce identical bytes.
 *
 * `hash3d` mixes three uint32 inputs (e.g. seed, primitive index, channel)
 *   into one uint32 of output entropy.
 * `hash_to_float`      maps uint32 → [-1, 1).
 * `hash_to_unit_float` maps uint32 → [0, 1).
 *============================================================================*/

#include <stdint.h>

uint32_t hash3d(uint32_t a, uint32_t b, uint32_t c);
float    hash_to_float(uint32_t h);
float    hash_to_unit_float(uint32_t h);

#endif /* PX4_SRC_LIB_TERRAIN_TERRAIN_H_ */
