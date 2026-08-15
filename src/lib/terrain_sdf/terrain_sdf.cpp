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
 * @file terrain_sdf.c
 *
 * Sphere tracer + analytic primitive SDFs for SIH world-aware sensors.
 *
 * Algorithm:  Hart 1996 sphere tracing over a min()-composed scene.
 * Primitives: Quilez 2008 closed-form signed-distance functions
 *             (box, vertical cylinder, sphere, horizontal plane).
 * Heightfield: a loose altitude-up SDF wrapping terrain() from
 *              `lib/terrain` — see scene_eval() for the contract.
 *
 * Frame: altitude-up everywhere (`sdf_vec3.alt` positive up). NED
 *        callers flip Z at the boundary. See terrain_sdf.h.
 *
 * References:
 *   Hart, "Sphere tracing: a geometric method for the antialiased
 *     ray tracing of implicit surfaces", The Visual Computer 12(10),
 *     1996.
 *   Quilez, "Distance functions",
 *     https://iquilezles.org/articles/distfunctions/, 2008.
 *
 * Constraints (matching `lib/terrain`):
 *   - Pure C, no PX4 headers, only <math.h> + <stdbool.h>.
 *   - No heap, no dynamic allocation. Scene lives in BSS.
 *   - No threading primitives; SIH calls this from one task.
 */

#include "terrain_sdf.h"
#include "scene.h"

#include "../terrain/terrain.h"

#include <math.h>
#include <stddef.h>

/*============================================================================
 * Tracer tuning constants.
 *
 * SDF_HIT_EPS — surface band: |scene_eval(p)| < HIT_EPS is a hit. 0.02 m
 *               (2 cm) is well below SIH rangefinder reporting resolution
 *               (1 cm), so a hit at this band reports the same integer
 *               distance the surface itself would.
 *
 * SDF_MAX_ITER — Hart 1996 step budget. At 64 iters with the MIN_STEP
 *                floor (0.05 m) the tracer can clear ≥ 3.2 m of stalled
 *                travel before bailing — covers tangent grazes against
 *                the heightfield where the loose wrapper isn't
 *                1-Lipschitz.
 *
 * SDF_SPHERE_TRACE_MIN_STEP — step floor. The heightfield SDF
 *                (`p.alt - terrain(p.n, p.e)`) is the vertical-distance
 *                approximation; on steep slopes it under-estimates the
 *                true Euclidean distance to the surface, meaning a
 *                naive step by `d` can stall arbitrarily close to but
 *                not at the surface. The floor guarantees forward
 *                progress; the stagnation counter caps the cost of
 *                grinding through near-tangent geometry.
 *============================================================================*/

#define SDF_HIT_EPS                   0.02f
#define SDF_MAX_ITER                  64
#define SDF_SPHERE_TRACE_MIN_STEP     0.05f
#define SDF_STAGNATION_LIMIT          3

/*============================================================================
 * Scene storage (static, BSS).
 *============================================================================*/

static sdf_prim_t s_scene[SDF_MAX_PRIMS];
static int        s_scene_count;

/* Procedural wall lattice toggle. When set, scene_eval() queries walls
 * lazily per-cell via scene_wall_at_cell() instead of from a materialized
 * list. See sdf_walls_set_enabled(). */
static bool       s_walls_enabled;

void sdf_scene_clear(void)
{
	s_scene_count = 0;
}

void sdf_walls_set_enabled(bool enabled)
{
	s_walls_enabled = enabled;
}

bool sdf_scene_add(const sdf_prim_t *p)
{
	if (p == NULL) {
		return false;
	}

	if (s_scene_count >= SDF_MAX_PRIMS) {
		return false;
	}

	s_scene[s_scene_count] = *p;
	s_scene_count++;
	return true;
}

/*============================================================================
 * Per-primitive SDF helpers (Quilez 2008).
 *
 * All take a query point in altitude-up world frame and return the
 * signed distance to the primitive surface — negative inside, positive
 * outside.
 *============================================================================*/

/* Box centred at `c` with local half-extents `h` (h.n, h.e, h.alt),
 * rotated about the altitude axis by yaw `(cos_yaw, sin_yaw)`.
 *
 * The closed-form box SDF is axis-aligned, so we transform the query
 * point into the wall's LOCAL frame first: subtract the centre, then
 * apply the inverse rotation about the altitude axis. The altitude
 * axis itself is invariant — yaw is purely an N-E rotation. With
 * `R(yaw)` being the forward rotation that produced the wall's
 * orientation, the inverse is `R(-yaw) = R^T`, which in 2D is
 *
 *   local_n =  cos_yaw * dn + sin_yaw * de
 *   local_e = -sin_yaw * dn + cos_yaw * de
 *
 * Cost: 4 muls + 2 adds + 2 subs per box per sphere-trace step. This
 * replaces the per-call sinf/cosf the naive form would need — the
 * precomputed pair (set by scene_wall_at_cell at generation time) is
 * the whole reason `sdf_prim_t` carries `cos_yaw` / `sin_yaw`. See
 * `terrain_sdf.h` on the hot-path budget.
 *
 * When the rotation pair is (1, 0) — i.e. no yaw — this collapses to
 * the original axis-aligned form (the local coordinates equal the
 * world deltas), so primitives that don't care about rotation can be
 * authored with `cos_yaw = 1, sin_yaw = 0` and pay nothing on the
 * hot path beyond the 6 wasted FLOPs.
 */
static inline float prim_sdf_box(sdf_vec3 p, sdf_vec3 c, sdf_vec3 h,
				 float cos_yaw, float sin_yaw)
{
	const float dn = p.n - c.n;
	const float de = p.e - c.e;
	const float local_n =  cos_yaw * dn + sin_yaw * de;
	const float local_e = -sin_yaw * dn + cos_yaw * de;

	float qn = fabsf(local_n)       - h.n;
	float qe = fabsf(local_e)       - h.e;
	float qa = fabsf(p.alt - c.alt) - h.alt;

	float outside_n = (qn > 0.f) ? qn : 0.f;
	float outside_e = (qe > 0.f) ? qe : 0.f;
	float outside_a = (qa > 0.f) ? qa : 0.f;

	float outside = sqrtf(outside_n * outside_n
			      + outside_e * outside_e
			      + outside_a * outside_a);

	float inside_max = qn;

	if (qe > inside_max) {
		inside_max = qe;
	}

	if (qa > inside_max) {
		inside_max = qa;
	}

	float inside = (inside_max < 0.f) ? inside_max : 0.f;

	return outside + inside;
}

/* Vertical cylinder centred at `c`, axis parallel to altitude axis.
 * `radius` = horizontal radius, `half_height` = half the altitude extent.
 */
static inline float prim_sdf_cylinder(sdf_vec3 p, sdf_vec3 c, float radius, float half_height)
{
	float dn = p.n - c.n;
	float de = p.e - c.e;
	float radial   = sqrtf(dn * dn + de * de) - radius;
	float vertical = fabsf(p.alt - c.alt) - half_height;

	float outside_r = (radial   > 0.f) ? radial   : 0.f;
	float outside_v = (vertical > 0.f) ? vertical : 0.f;

	float outside = sqrtf(outside_r * outside_r + outside_v * outside_v);

	float inside_max = (radial > vertical) ? radial : vertical;
	float inside = (inside_max < 0.f) ? inside_max : 0.f;

	return outside + inside;
}

/* Sphere centred at `c` with radius `r`. */
static inline float prim_sdf_sphere(sdf_vec3 p, sdf_vec3 c, float r)
{
	float dn = p.n   - c.n;
	float de = p.e   - c.e;
	float da = p.alt - c.alt;
	return sqrtf(dn * dn + de * de + da * da) - r;
}

/* Horizontal infinite plane at altitude `plane_alt`, normal pointing up.
 * Above plane is positive (outside the half-space, in scene-union terms). */
static inline float prim_sdf_plane(sdf_vec3 p, float plane_alt)
{
	return p.alt - plane_alt;
}

/* Dispatch one primitive by type. */
static inline float prim_sdf(sdf_vec3 p, const sdf_prim_t *prim)
{
	switch (prim->type) {
	case SDF_PRIM_BOX:
		return prim_sdf_box(p, prim->center, prim->extent,
				    prim->cos_yaw, prim->sin_yaw);

	case SDF_PRIM_CYLINDER:
		return prim_sdf_cylinder(p, prim->center, prim->extent.n, prim->extent.alt);

	case SDF_PRIM_SPHERE:
		return prim_sdf_sphere(p, prim->center, prim->extent.n);

	case SDF_PRIM_PLANE:
		return prim_sdf_plane(p, prim->center.alt);

	default:
		/* Unknown type: contribute infinity (i.e. no surface). The
		 * union min() ignores it. */
		return INFINITY;
	}
}

/*============================================================================
 * scene_eval — min()-union of all primitives + heightfield SDF.
 *
 * The heightfield SDF is `p.alt - terrain(p.n, p.e)` — i.e. the
 * vertical distance from the point to the heightfield surface beneath
 * it. This is NOT the true Euclidean distance on a sloped surface
 * (the closest point on the heightfield is in general not directly
 * below); it is a loose lower bound only when the heightfield is
 * locally flat. On steep slopes the tracer can stall near the surface
 * — that is what SDF_SPHERE_TRACE_MIN_STEP + the stagnation counter
 * compensate for.
 *
 * We do not use terrain_gradient() to tighten the bound — Quilez
 * "terrainmarching" uses gradient-based step length, but it requires
 * a Lipschitz constant on the gradient which the fBm gradient
 * doesn't have a tight bound on (see the original lib/terrain
 * cone-marcher comments). The fixed-floor approach trades a few extra
 * iterations for simpler correctness.
 *============================================================================*/

/*============================================================================
 * scene_walls_sdf — loose, local wall-field SDF.
 *
 * The obstacle-mode analogue of terrain(). Walls are not a stored list; they
 * are a deterministic function of (seed, cell) — scene_wall_at_cell() is to
 * walls what terrain() is to hills. So we never iterate a list: at a sample
 * point we look only at the lattice cells local to it, exactly the way
 * terrain() reads only the noise under the point.
 *
 * A wall reaches at most ~one cell beyond its own cell, so a fixed
 * WALL_QUERY_RADIUS_CELLS neighbourhood around the sample cell contains any
 * wall whose surface can be near the point. Most cells are empty and
 * short-circuit on the cheap presence hash; only a couple of cells hold a
 * wall, so the cost is a couple of box evaluations per step regardless of how
 * many walls the seed places anywhere in the world.
 *
 * When no local wall is found we return WALL_CELL_SPACING_M as a guaranteed-
 * clear bound rather than the true distance to some distant wall: any wall
 * outside the searched neighbourhood is farther than this, so the sphere
 * tracer re-evaluates the next cell before it could step past one. This is
 * the same loose-but-safe contract terrain's vertical-distance SDF relies on
 * (the min-step floor + stagnation counter absorb the looseness). It is
 * deliberately NOT a tight global "nearest wall" — that would force a
 * content-dependent search, which is the whole cost we are removing.
 *============================================================================*/
#define WALL_QUERY_RADIUS_CELLS 1

static float scene_walls_sdf(sdf_vec3 p)
{
	if (!s_walls_enabled) {
		return INFINITY;
	}

	/* Cell (i, j) is centred at (i, j) * WALL_CELL_SPACING_M. */
	const int ci = (int)lrintf(p.n / WALL_CELL_SPACING_M);
	const int cj = (int)lrintf(p.e / WALL_CELL_SPACING_M);

	/* Guaranteed-clear bound with no local wall: with a 1-cell search any
	 * wall outside it is >= ~0.75 cell beyond the searched ring, so half a
	 * cell is a safe step that re-evaluates the next ring before reaching
	 * one. */
	float d = 0.5f * WALL_CELL_SPACING_M;

	for (int di = -WALL_QUERY_RADIUS_CELLS; di <= WALL_QUERY_RADIUS_CELLS; ++di) {
		const int i = ci + di;

		if (i < -WALL_CELL_RADIUS_CELLS || i > WALL_CELL_RADIUS_CELLS) {
			continue;
		}

		for (int dj = -WALL_QUERY_RADIUS_CELLS; dj <= WALL_QUERY_RADIUS_CELLS; ++dj) {
			const int j = cj + dj;

			if (j < -WALL_CELL_RADIUS_CELLS || j > WALL_CELL_RADIUS_CELLS) {
				continue;
			}

			sdf_prim_t wall;

			if (scene_wall_at_cell(i, j, &wall)) {
				const float dw = prim_sdf(p, &wall);

				if (dw < d) {
					d = dw;
				}
			}
		}
	}

	return d;
}

/*
 * The continuous part of the scene: the heightfield plus the procedural
 * wall lattice. Both are fields rather than objects, so a ray can only
 * find them by marching.
 */
static float scene_field_sdf(sdf_vec3 p)
{
	float h = terrain(p.n, p.e);
	float d = p.alt - h;

	const float dwalls = scene_walls_sdf(p);

	if (dwalls < d) {
		d = dwalls;
	}

	return d;
}

static float scene_eval(sdf_vec3 p)
{
	float d = scene_field_sdf(p);

	for (int i = 0; i < s_scene_count; ++i) {
		float di = prim_sdf(p, &s_scene[i]);

		if (di < d) {
			d = di;
		}
	}

	return d;
}

/*============================================================================
 * scene_prims_ray - exact ray/primitive intersection over the scene list.
 *
 * Marching finds a surface by stepping the distance to the NEAREST surface
 * in any direction. Indoors that collapses: a ray down a 24 m corridor steps
 * 12 m at a time no matter how far the clear air ahead runs, because a side
 * wall is what bounds the step. Run out of SDF_MAX_ITER and the tracer
 * returns max_t, which is the same value SIH publishes to mean "nothing in
 * range" - an exhausted ray and a clear ray become indistinguishable.
 *
 * Boxes do not need marching. A ray/box intersection is closed form, so the
 * answer is exact and costs the same whatever the geometry looks like. This
 * applies to any materialised primitive rather than to one scene mode.
 *
 * Slab method in the box's local frame, using the same inverse yaw rotation
 * prim_sdf_box() applies to a point. Returns the nearest forward hit within
 * max_t, or max_t when nothing is hit.
 *============================================================================*/
static float box_ray_hit(sdf_vec3 o, sdf_vec3 d, const sdf_prim_t *b, float max_t)
{
	/* Rotate the ray into the box's local frame. Altitude is invariant. */
	const float dn = o.n - b->center.n;
	const float de = o.e - b->center.e;

	const float on =  b->cos_yaw * dn + b->sin_yaw * de;
	const float oe = -b->sin_yaw * dn + b->cos_yaw * de;
	const float oa = o.alt - b->center.alt;

	const float rn =  b->cos_yaw * d.n + b->sin_yaw * d.e;
	const float re = -b->sin_yaw * d.n + b->cos_yaw * d.e;
	const float ra = d.alt;

	const float org[3] = { on, oe, oa };
	const float dir[3] = { rn, re, ra };
	const float ext[3] = { b->extent.n, b->extent.e, b->extent.alt };

	float t_near = 0.f;
	float t_far  = max_t;

	for (int a = 0; a < 3; ++a) {
		if (fabsf(dir[a]) < 1e-9f) {
			/* Parallel to this slab: miss unless already inside it. */
			if (org[a] < -ext[a] || org[a] > ext[a]) {
				return max_t;
			}

			continue;
		}

		const float inv = 1.f / dir[a];
		float t0 = (-ext[a] - org[a]) * inv;
		float t1 = (ext[a] - org[a]) * inv;

		if (t0 > t1) {
			const float tmp = t0;
			t0 = t1;
			t1 = tmp;
		}

		if (t0 > t_near) { t_near = t0; }

		if (t1 < t_far)  { t_far  = t1; }

		if (t_near > t_far) {
			return max_t;
		}
	}

	return t_near;
}

static float scene_prims_ray(sdf_vec3 o, sdf_vec3 d, float max_t)
{
	float best = max_t;

	for (int i = 0; i < s_scene_count; ++i) {
		/* Only boxes have a closed form here. Anything else stays with
		 * the marcher, which still runs against the field below. */
		if (s_scene[i].type != SDF_PRIM_BOX) {
			continue;
		}

		const float t = box_ray_hit(o, d, &s_scene[i], best);

		if (t < best) {
			best = t;
		}
	}

	return best;
}

/*============================================================================
 * sdf_sphere_trace — Hart 1996 with origin guard + min-step + stagnation.
 *
 * Origin guard: if scene_eval(origin) < 0 the ray starts inside scene
 * geometry. Return -1.f sentinel per the API contract; callers map
 * this to "vehicle clipping geometry".
 *
 * Min-step floor: each iteration advances by max(d, MIN_STEP). This
 * keeps the tracer making forward progress against the loose
 * heightfield SDF and against tangent grazes on primitive surfaces.
 *
 * Stagnation break: if MIN_STEP fires SDF_STAGNATION_LIMIT iterations
 * in a row the tracer has been within MIN_STEP of a surface for
 * multiple steps without crossing the HIT_EPS band — practically a
 * grazing tangent. Return the current t as a best-effort hit; the
 * caller treats it as a contact within MIN_STEP. The counter resets
 * any iteration that advances by more than MIN_STEP.
 *============================================================================*/

float sdf_sphere_trace(sdf_vec3 origin, sdf_vec3 dir_unit, float max_t)
{
	if (max_t <= 0.f) {
		return 0.f;
	}

	if (scene_eval(origin) < 0.f) {
		return -1.f;
	}

	/*
	 * Boxes are solved exactly, then the field is marched only as far as
	 * the nearest box. Walls stop being something the marcher has to
	 * creep up on, so a corridor costs the same per ray as open air and
	 * no ray can exhaust its budget and report clear air where a wall is.
	 */
	const float t_prim = scene_prims_ray(origin, dir_unit, max_t);
	const float march_limit = (t_prim < max_t) ? t_prim : max_t;

	float t = 0.f;
	int   stagnant = 0;

	for (int i = 0; i < SDF_MAX_ITER; ++i) {
		sdf_vec3 p = {
			origin.n   + t * dir_unit.n,
			origin.e   + t * dir_unit.e,
			origin.alt + t * dir_unit.alt,
		};

		/*
		 * Step by the full scene, boxes included, even though the boxes
		 * were already solved exactly above. The step length must stay
		 * a lower bound on the distance to ANY surface, and the
		 * heightfield SDF is a vertical distance rather than a
		 * Euclidean one, so on a slope it overestimates and a longer
		 * step can jump straight over the hillside. Dropping the boxes
		 * from this term lengthened the step and did exactly that,
		 * moving a terrain hit at 120.70 m out to 122.03 m.
		 *
		 * The exact box answer is still what gets returned, via
		 * march_limit. This term only decides how far it is safe to
		 * advance.
		 */
		float d = scene_eval(p);

		if (d < SDF_HIT_EPS) {
			return t;
		}

		float step;

		if (d < SDF_SPHERE_TRACE_MIN_STEP) {
			step = SDF_SPHERE_TRACE_MIN_STEP;
			stagnant++;

			if (stagnant >= SDF_STAGNATION_LIMIT) {
				return t;
			}

		} else {
			step = d;
			stagnant = 0;
		}

		t += step;

		if (t >= march_limit) {
			/* Either a box is nearer than anything the field holds,
			 * or nothing is in range at all. */
			return march_limit;
		}
	}

	return march_limit;
}
