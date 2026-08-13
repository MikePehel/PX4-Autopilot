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
 * @file terrain.c
 *
 * Procedural terrain function + analytical near-vertical raycast for SIH.
 *
 * Math reference: Quilez, "fBm with derivatives + slope erosion divisor"
 * (iquilezles.org/articles/morenoise/, 2008).
 *
 * Constraints (per Shared Source Contract):
 *   - Pure C, no PX4 headers, no STL.
 *   - Only <math.h> essentials: floorf, fabsf, tanf.
 *   - No heap, no dynamic allocation.
 *   - Bit-exact integer hash across ARM / x86_64 / wasm32.
 *
 * The slope-attenuated divisor `1 / (1 + epsilon * |grad|^2)` damps high
 * octaves where slope already accumulated, producing terrain that
 * "smooths out" on steep faces (a cheap erosion analogue). The running
 * `(grad_n, grad_e)` accumulator inside the loop IS the analytic
 * gradient w.r.t. (n, e), so terrain_gradient() costs no extra work.
 *
 * Arbitrary-direction raycasts against the heightfield + obstacles live
 * in lib/terrain_sdf (sphere tracer over SDF primitives); raycast() in
 * this library is the analytical fast-path for near-vertical downward
 * beams only.
 */

#include "terrain.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/*============================================================================
 * Static configuration state.
 *
 * Set by terrain_set_params() and read (never written) by every
 * evaluation. This is the only mutable state in the library.
 *============================================================================*/

/* Degrees-to-radians constant. Defined locally so the file does not
 * depend on a non-portable M_PI from <math.h>. */
#define TERRAIN_DEG_TO_RAD   (0.01745329251994329577f)

/* Defaults live on the members rather than in a trailing initialiser, so a
 * field and its default cannot drift apart when one is reordered. */
static struct {
	float    amp         = 0.f;
	float    wavelength  = 200.f;
	int      oct         = 6;
	float    hurst       = 0.7f;
	float    erosion     = 1.f;
	int      seed        = 0;
	float    home_n      = 0.f;
	float    home_e      = 0.f;
	float    plane_deg   = 0.f;   /* deterministic planar mode: 0 = fBm, else angle [deg] */

	/* Derived; recomputed in set_params(). */
	float    amp_decay   = 0.6155722f;  /* 2^(-hurst), per-octave amplitude factor */
	float    home_offset = 0.f;         /* raw_eval(home_n, home_e) — subtracted in terrain() */
	float    plane_tan   = 0.f;         /* tanf(plane_deg * π/180), precomputed for the hot path */
} s_params;

/* Compile-time square of the flat takeoff-zone radius. Cached so the
 * hot path compares r² against fr² without a runtime `sqrtf`. */
#define FLAT_R_SQ_M  ((FLAT_R_M) * (FLAT_R_M))

/* Installed heightfield grid, or all-zero when none. The sample array is
 * borrowed from the caller (see terrain_set_grid) — this library never
 * allocates, copies, or frees it. A NULL `samples` means "no grid", which
 * is the state grid_eval() short-circuits on. */
static terrain_grid_t s_grid;

/*============================================================================
 * Integer hash + hash-to-float.
 *
 * Murmur3-finalizer style. Bit-exact for any inputs across any compiler
 * that respects 32-bit two's-complement unsigned arithmetic — which is
 * every platform PX4 supports.
 *============================================================================*/

static inline uint32_t hash2d(int32_t ix, int32_t iy, int32_t seed)
{
	uint32_t h = (uint32_t)ix * 0x27d4eb2du;
	h ^= (uint32_t)iy * 0x9e3779b1u;
	h ^= (uint32_t)seed * 0x85ebca6bu;
	h ^= h >> 16;
	h *= 0x85ebca6bu;
	h ^= h >> 13;
	h *= 0xc2b2ae35u;
	h ^= h >> 16;
	return h;
}

/* Map a 32-bit hash deterministically to [-1, 1) using its upper 24 bits.
 * Avoids any union punning so we can stay strict-aliasing-clean.
 *
 * Exposed publicly (declared in terrain.h) so SDF scene generators can
 * derive signed lattice values from the shared seed. */
float hash_to_float(uint32_t h)
{
	int32_t s = (int32_t)((h >> 8) & 0x00FFFFFFu) - 0x00800000;
	return (float)s * (1.f / (float)0x00800000);
}

/* Map a 32-bit hash deterministically to [0, 1) using its upper 24 bits.
 * Companion to `hash_to_float` for callers that want an unsigned uniform
 * sample (e.g. wall-angle ∈ [0, 2π), wall-distance lerp into an annulus). */
float hash_to_unit_float(uint32_t h)
{
	return (float)((h >> 8) & 0x00FFFFFFu) * (1.f / (float)0x01000000);
}

/* Three-input integer hash with the same bit-exact, cross-platform
 * contract as `hash2d`. Murmur3-finalizer recipe extended with a third
 * input multiplier. Exposed publicly so SDF scene generators can derive
 * deterministic primitives from (seed, index, channel) tuples. */
uint32_t hash3d(uint32_t a, uint32_t b, uint32_t c)
{
	uint32_t h = a * 0x27d4eb2du;
	h ^= b * 0x9e3779b1u;
	h ^= c * 0x85ebca6bu;
	h ^= h >> 16;
	h *= 0x85ebca6bu;
	h ^= h >> 13;
	h *= 0xc2b2ae35u;
	h ^= h >> 16;
	return h;
}

/*============================================================================
 * Value noise with analytic derivative.
 *
 * Quintic (Perlin) smoothstep gives C2 continuity, so the gradient
 * itself is C1 — the erosion divisor doesn't see kinks at cell
 * boundaries.
 *
 *   smooth(t)  = t^3 (t (6 t - 15) + 10)
 *   smooth'(t) = 30 t^2 (t (t - 2) + 1)
 *============================================================================*/

static void value_noise_d(float x, float y, int32_t seed,
			  float *out_n, float *out_dx, float *out_dy)
{
	float xi_f = floorf(x);
	float yi_f = floorf(y);
	float xf   = x - xi_f;
	float yf   = y - yi_f;

	int32_t ix = (int32_t)xi_f;
	int32_t iy = (int32_t)yi_f;

	float u  = xf * xf * xf * (xf * (xf * 6.f - 15.f) + 10.f);
	float du = 30.f * xf * xf * (xf * (xf - 2.f) + 1.f);
	float v  = yf * yf * yf * (yf * (yf * 6.f - 15.f) + 10.f);
	float dv = 30.f * yf * yf * (yf * (yf - 2.f) + 1.f);

	float a = hash_to_float(hash2d(ix,     iy,     seed));
	float b = hash_to_float(hash2d(ix + 1, iy,     seed));
	float c = hash_to_float(hash2d(ix,     iy + 1, seed));
	float d = hash_to_float(hash2d(ix + 1, iy + 1, seed));

	/* Bilinear with smooth blend:
	 *   n(u, v) = a + (b - a) u + (c - a) v + (a - b - c + d) u v
	 */
	float k0 = a;
	float k1 = b - a;
	float k2 = c - a;
	float k3 = a - b - c + d;

	*out_n  = k0 + k1 * u + k2 * v + k3 * u * v;
	*out_dx = (k1 + k3 * v) * du;
	*out_dy = (k2 + k3 * u) * dv;
}

/*============================================================================
 * Multi-octave fBm with rotation between octaves + slope-attenuated
 * divisor (Quilez 2008).
 *
 * The Quilez rotation matrix has determinant 4 and operator norm ~2,
 * which approximates "frequency doubles per octave" while keeping the
 * orientation different per octave so octaves don't visibly align.
 *
 *   R = [[ 1.6,  1.2],
 *        [-1.2,  1.6]]
 *
 * Cumulative chain rule for the gradient w.r.t. (n, e):
 *
 *   At octave i, the noise input is (x_i, y_i) = C_i * k * (n, e),
 *   where C_i is the cumulative rotation (C_0 = I) and k = 1/wavelength.
 *
 *   d noise_i / d n = (∂x_i/∂n) * d_x noise + (∂y_i/∂n) * d_y noise
 *                  = k * (C_i[0,0] * dnx + C_i[1,0] * dny)
 *   d noise_i / d e = k * (C_i[0,1] * dnx + C_i[1,1] * dny)
 *
 * The erosion divisor `1 / (1 + erosion * |grad_cum|^2)` uses the
 * snapshot of |grad_cum| from previously-accumulated octaves. We
 * intentionally do NOT differentiate through the divisor itself (that
 * would require an iterative fixed-point and is not standard Quilez).
 *============================================================================*/

static float raw_eval(float n, float e, float *out_dn, float *out_de)
{
	if (fabsf(s_params.amp) < 1e-6f) {
		if (out_dn) {
			*out_dn = 0.f;
		}

		if (out_de) {
			*out_de = 0.f;
		}

		return 0.f;
	}

	const float k = 1.f / s_params.wavelength;
	float x = n * k;
	float y = e * k;

	/* Cumulative rotation C_i (C_0 = I). */
	float c00 = 1.f, c01 = 0.f;
	float c10 = 0.f, c11 = 1.f;

	const float R00 =  1.6f, R01 =  1.2f;
	const float R10 = -1.2f, R11 =  1.6f;

	float h  = 0.f;
	float dn = 0.f;
	float de = 0.f;
	float amp = s_params.amp;

	for (int i = 0; i < s_params.oct; ++i) {
		float nv, ndx, ndy;
		value_noise_d(x, y, s_params.seed + i, &nv, &ndx, &ndy);

		/* Chain rule: gradient w.r.t. (n, e), not (x, y). */
		float gn = (c00 * ndx + c10 * ndy) * k;
		float ge = (c01 * ndx + c11 * ndy) * k;

		/* Erosion divisor: high cumulative slope damps further octaves.
		 * The denom is treated as a constant w.r.t. (n, e) — see header
		 * comment above. */
		float denom = 1.f + s_params.erosion * (dn * dn + de * de);
		float inv_denom = 1.f / denom;

		h  += amp * nv  * inv_denom;
		dn += amp * gn  * inv_denom;
		de += amp * ge  * inv_denom;

		/* Advance: rotate input and update cumulative C = R * C. */
		float xr = R00 * x + R01 * y;
		float yr = R10 * x + R11 * y;
		x = xr;
		y = yr;

		float nc00 = R00 * c00 + R01 * c10;
		float nc01 = R00 * c01 + R01 * c11;
		float nc10 = R10 * c00 + R11 * c10;
		float nc11 = R10 * c01 + R11 * c11;
		c00 = nc00;
		c01 = nc01;
		c10 = nc10;
		c11 = nc11;

		amp *= s_params.amp_decay;
	}

	if (out_dn) {
		*out_dn = dn;
	}

	if (out_de) {
		*out_de = de;
	}

	return h;
}

/*============================================================================
 * Heightfield grid sampling — bilinear with toroidal wrap.
 *
 * Wrap rather than clamp: lib/terrain_sdf's sphere tracer marches to
 * max_t with no bounds check, so the heightfield has to be defined at
 * every (N, E). Clamping would extrude the boundary row outward forever
 * and the obstacle ring would read it as a wall.
 *
 * The bilinear patch over one cell is
 *
 *   h(tx, ty) = a + b*tx + c*ty + d*tx*ty
 *
 * with a = h00, b = h10 - h00, c = h01 - h00, d = h00 - h10 - h01 + h11.
 * Its partials are exact and cost two multiplies each, so the gradient
 * comes out of the same four samples the height did — the same "gradient
 * is free" property the fBm accumulator has.
 *
 * Only C0 across cell boundaries: the height is continuous but the slope
 * steps at each grid line. See the contract in terrain.h.
 *============================================================================*/

/* Positive modulo. C's % truncates toward zero, so a negative cell index
 * would otherwise land outside the sample array. */
static inline int wrap_idx(int i, int n)
{
	i %= n;
	return (i < 0) ? (i + n) : i;
}

static float grid_eval(float n_world, float e_world, float *out_dn, float *out_de)
{
	if (s_grid.samples == NULL) {
		*out_dn = 0.f;
		*out_de = 0.f;
		return 0.f;
	}

	const float inv_s = 1.f / s_grid.spacing_m;
	const float fy = (n_world - s_grid.origin_n) * inv_s;
	const float fx = (e_world - s_grid.origin_e) * inv_s;

	const float y0f = floorf(fy);
	const float x0f = floorf(fx);
	const float ty  = fy - y0f;
	const float tx  = fx - x0f;

	const int nx = (int)s_grid.nx;
	const int ny = (int)s_grid.ny;

	const int y0 = wrap_idx((int)y0f, ny);
	const int x0 = wrap_idx((int)x0f, nx);
	const int y1 = (y0 + 1 == ny) ? 0 : (y0 + 1);
	const int x1 = (x0 + 1 == nx) ? 0 : (x0 + 1);

	const float h00 = (float)s_grid.samples[(y0 * nx) + x0];
	const float h10 = (float)s_grid.samples[(y0 * nx) + x1];   /* +east  */
	const float h01 = (float)s_grid.samples[(y1 * nx) + x0];   /* +north */
	const float h11 = (float)s_grid.samples[(y1 * nx) + x1];

	const float b = h10 - h00;
	const float c = h01 - h00;
	const float d = (h00 - h10 - h01) + h11;

	*out_dn = (c + (d * tx)) * inv_s;
	*out_de = (b + (d * ty)) * inv_s;

	return h00 + (b * tx) + (c * ty) + (d * tx * ty);
}

/* Recompute the home offset from whatever sources are currently active.
 * Both terrain_set_params() and terrain_set_grid() call this so that
 * terrain(0, 0) == 0 holds after either one changes. */
static void recompute_home_offset(void)
{
	float dn, de, gdn, gde;
	s_params.home_offset =
		raw_eval(s_params.home_n, s_params.home_e, &dn, &de)
		+ grid_eval(s_params.home_n, s_params.home_e, &gdn, &gde);
}

void terrain_set_grid(const terrain_grid_t *grid)
{
	/* Reject anything that would make grid_eval() read out of bounds or
	 * divide by zero. A failed file load that left the struct partly
	 * populated must degrade to "no grid", never to garbage terrain. */
	if (grid == NULL || grid->samples == NULL
	    || grid->nx < 2u || grid->ny < 2u
	    || !(grid->spacing_m > 0.f)) {
		s_grid.samples = NULL;

	} else {
		s_grid = *grid;
	}

	recompute_home_offset();
}

/* Public terrain() and terrain_gradient() apply the home offset so that
 * terrain(home_n - home_n, home_e - home_e) = terrain(0, 0) = 0.
 *
 * When planar mode is engaged (plane_deg != 0), the fBm pipeline is
 * short-circuited and the surface is a uniform half-space tilted toward
 * +N: h(N, E) = tan(plane_deg) * N. terrain(0, 0) == 0 is preserved by
 * construction without any home-offset machinery. Planar mode takes
 * precedence over the flat takeoff zone — the two are independent
 * mechanisms.
 *
 * Flat takeoff zone: when FLAT_R_M > 0 (the
 * compile-time radius from terrain.h), both terrain() and
 * terrain_gradient() return 0 / (0, 0) inside the disk of radius
 * FLAT_R_M around home. The check uses r² < FLAT_R_M² so there is no
 * sqrtf in the hot path. The transition is a hard step — no smooth
 * ramp — which is fine for vertical takeoff/landing inside the disk
 * (the only scenario the disk is meant to serve) and is absorbed by
 * the spring-damper landing gear for taxi-across cases.
 *
 * When FLAT_R_M is 0, the disk check optimises out at compile time
 * and the functions reduce to pure fBm with no flat zone. */

float terrain(float north_m, float east_m)
{
	if (fabsf(s_params.plane_deg) > 1e-6f) {
		(void)east_m;
		return s_params.plane_tan * north_m;
	}

	/* Flat takeoff zone — hard cutoff. With FLAT_R_M = 0 the
	 * disk check is `r² < 0` which is always false, and the optimiser
	 * elides the whole branch. With FLAT_R_M > 0 the disk is enforced
	 * via an r² compare so no sqrtf appears in the hot path. */
	if (north_m * north_m + east_m * east_m < FLAT_R_SQ_M) {
		return 0.f;
	}

	/* Grid and fBm are additive. With no grid installed grid_eval() is a
	 * short-circuit zero; with amp == 0 raw_eval() is. So "DEM only",
	 * "fBm only", and "real landform plus procedural detail" are all the
	 * same code path with no branch on the hot line. */
	const float n_world = north_m + s_params.home_n;
	const float e_world = east_m  + s_params.home_e;

	float dn, de, gdn, gde;
	return raw_eval(n_world, e_world, &dn, &de)
	       + grid_eval(n_world, e_world, &gdn, &gde)
	       - s_params.home_offset;
}

void terrain_gradient(float north_m, float east_m, float *dnorth, float *deast)
{
	/* dnorth and deast are required — all in-tree callers pass valid
	 * pointers. If a future caller wants only one component, pass a
	 * stack-local float for the other. */

	if (fabsf(s_params.plane_deg) > 1e-6f) {
		*dnorth = s_params.plane_tan;
		*deast  = 0.f;
		return;
	}

	if (north_m * north_m + east_m * east_m < FLAT_R_SQ_M) {
		*dnorth = 0.f;
		*deast  = 0.f;
		return;
	}

	const float n_world = north_m + s_params.home_n;
	const float e_world = east_m  + s_params.home_e;

	float dn_raw = 0.f;
	float de_raw = 0.f;
	(void)raw_eval(n_world, e_world, &dn_raw, &de_raw);

	/* Same additive composition as terrain(): the grid's bilinear slope
	 * and the fBm slope sum, so layering noise over a coarse DEM gives
	 * the contact model a gradient that reflects both. */
	float dn_grid = 0.f;
	float de_grid = 0.f;
	(void)grid_eval(n_world, e_world, &dn_grid, &de_grid);

	*dnorth = dn_raw + dn_grid;
	*deast  = de_raw + de_grid;
}

/*============================================================================
 * Raycast — analytical fast-path only.
 *
 * Near-vertical downward beams only — the downward-rangefinder case.
 * Arbitrary-direction beams (forward / side / up sensors, the obstacle
 * ring) go through sdf_sphere_trace in lib/terrain_sdf, which composes
 * the heightfield with a primitive-SDF scene.
 *
 * The intersection solved here is
 *
 *   origin_alt + dir_alt * t = terrain(origin_n + dir_n * t,
 *                                       origin_e + dir_e * t)
 *
 * approximated as
 *
 *   t = (origin_alt - terrain(origin_n, origin_e)) / (-dir_alt)
 *
 * because over a ≤5°-off-vertical projection the terrain column under
 * the origin doesn't shift enough for the lateral approximation to
 * matter at SIH-rangefinder altitudes. Exact for straight-down
 * (dir_n = dir_e = 0); sub-sensor-noise (<< 1 cm) at the tilts a
 * level-vehicle downward beam actually exhibits.
 *
 * For beams outside the analytical envelope (tilt > ~5° off vertical,
 * including any upward beam) this function returns max_t (miss) — the
 * caller's signal to re-issue via sdf_sphere_trace. See the header
 * comment on raycast() for the full contract.
 *============================================================================*/

#define TERRAIN_RAYCAST_HIT_EPSILON     0.05f     /* metres of vertical clearance */
#define TERRAIN_TILT_ANALYTICAL_COS     0.9962f   /* cos(5 deg) */

float raycast(float origin_n, float origin_e, float origin_alt,
	      float dir_n, float dir_e, float dir_alt, float max_t)
{
	(void)dir_n;
	(void)dir_e;

	if (max_t <= 0.f) {
		return 0.f;
	}

	/* `cos_tilt` here is the cosine of the angle between the ray and
	 * the local DOWN axis. In altitude-up form, dir_alt is positive
	 * up, so a downward beam has dir_alt < 0 and the cosine against
	 * down is -dir_alt. Out of envelope when the beam is more than
	 * ~5° off straight down OR pointing up (cos_tilt < threshold). */
	const float cos_tilt = -dir_alt;

	if (cos_tilt < TERRAIN_TILT_ANALYTICAL_COS) {
		/* Out of envelope: route this beam through sdf_sphere_trace
		 * in lib/terrain_sdf. */
		return max_t;
	}

	float th = terrain(origin_n, origin_e);
	float t  = (origin_alt - th) / cos_tilt;

	if (t < 0.f) {
		return max_t;
	}

	if (t > max_t) {
		return max_t;
	}

	return t;
}

/*============================================================================
 * Configuration.
 *
 * Recomputes the derived values (home_offset, plane_tan) so the hot
 * path never has to. amp_decay lives in the static initializer because
 * the Hurst exponent it derives from is now hardcoded.
 *============================================================================*/

/* The fBm shape knobs (oct, hurst, erosion, and the derived amp_decay)
 * are now hard-coded in the s_params static initializer at the top of
 * this file. They are deep noise-generation internals with no operational
 * meaning to a pilot, so they are not exposed as runtime params. Anyone
 * needing to retune them edits the initializer and rebuilds.
 */
void terrain_set_params(float amp, float wavelength, int seed,
			float home_n, float home_e,
			float plane_deg)
{
	s_params.amp        = amp;
	s_params.wavelength = (wavelength > 1e-3f) ? wavelength : 1e-3f;
	s_params.seed       = seed;
	s_params.home_n     = home_n;
	s_params.home_e     = home_e;
	s_params.plane_deg  = plane_deg;
	s_params.plane_tan  = tanf(plane_deg * TERRAIN_DEG_TO_RAD);

	/* Home offset: recompute so terrain(0, 0) == 0 after this call.
	 * Samples raw_eval/grid_eval directly (not terrain()) since terrain()
	 * would subtract the stale offset. */
	recompute_home_offset();
}

unsigned int terrain_seed_lookup_table_size(void)
{
	/* fBm is hash-based and costs no table. A grid installed via
	 * terrain_set_grid() is borrowed, not owned, but reporting its size
	 * is what makes this accessor useful. */
	if (s_grid.samples == NULL) {
		return 0u;
	}

	return (unsigned int)s_grid.nx * (unsigned int)s_grid.ny
	       * (unsigned int)sizeof(int16_t);
}

int terrain_get_seed(void)
{
	return s_params.seed;
}
