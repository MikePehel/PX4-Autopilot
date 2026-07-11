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
 * @file terrain_sdf.h
 *
 * Signed-distance-field (SDF) sphere tracer for SIH world-aware sensors.
 *
 * Companion library to `lib/terrain`. Where `lib/terrain` handles the
 * procedural heightfield + a near-vertical analytical raycast, this
 * library handles arbitrary-direction beams against a `min()`-composed
 * scene of analytic primitive SDFs (`sdf_box`, `sdf_cylinder`,
 * `sdf_sphere`, `sdf_plane`) plus a loose heightfield SDF wrapping
 * `terrain()`. The traversal algorithm is Hart 1996 sphere tracing:
 * step along the ray by the SDF value at each iterate; that step is
 * guaranteed not to cross the surface because the SDF lower-bounds the
 * Euclidean distance to it.
 *
 * Used by SIH's `send_obstacle_distance()` (72-bin obstacle ring) and the
 * non-downward branches of `send_dist_snsr()` (forward / side / up
 * rangefinders) so they can see static man-made geometry that doesn't
 * live in the procedural heightfield.
 *
 * ## Frame convention (pinned — altitude-up everywhere)
 *
 * All coordinates in this library use the **altitude-up** convention:
 *
 * - `sdf_vec3.n` — north metres (NED north axis)
 * - `sdf_vec3.e` — east  metres (NED east  axis)
 * - `sdf_vec3.alt` — altitude metres, **positive up**
 *
 * The altitude axis is the negation of the NED "down" axis. A ray
 * pointing toward the ground has `dir_unit.alt < 0`; a ray pointing
 * skyward has `dir_unit.alt > 0`. Callers that hold their state in NED
 * with positive-down must flip the Z sign at the boundary
 * (`sdf_vec3.alt = -ned.d`).
 *
 * This convention matches the standard SDF / computer-graphics
 * convention (Quilez 2008) and matches `lib/terrain`'s height return
 * (`terrain()` returns altitude, positive up). It does NOT match raw
 * NED state. The convention is enforced at the library boundary; the
 * library itself never sees a "down" component.
 *
 * ## Scene model
 *
 * The scene is a fixed-capacity (`SDF_MAX_PRIMS`) flat list of analytic
 * primitives, evaluated with `min()` composition (union semantics). No
 * dynamic allocation; the list lives in BSS. Primitives are added with
 * `sdf_scene_add()` and the list is reset with `sdf_scene_clear()`.
 *
 * The heightfield SDF (an altitude-up wrapper around `terrain()`) is
 * always part of the scene; callers do not need to add it explicitly.
 * The heightfield wrapper is loose (not strictly 1-Lipschitz on steep
 * slopes), which is why the tracer floors its step at
 * `SDF_SPHERE_TRACE_MIN_STEP` — see the contract on `sdf_sphere_trace`.
 *
 * ## Primitive semantics
 *
 * Each `sdf_prim_t` is `{ type, center, extent, cos_yaw, sin_yaw }`. The
 * `extent` channels encode primitive-specific dimensions:
 *
 *   `SDF_PRIM_BOX`      — box rotated about the altitude axis by yaw.
 *                         `extent` = half-widths along the wall's
 *                         LOCAL (rotated) (n, e, alt) axes —
 *                         `extent.n` is the half-length along the long
 *                         axis, `extent.e` the half-thickness across
 *                         it, `extent.alt` the half-height. The
 *                         primitive's `cos_yaw` / `sin_yaw` are the
 *                         precomputed N-E-plane rotation that maps
 *                         world coordinates into the wall's local frame
 *                         before the axis-aligned box SDF is evaluated.
 *   `SDF_PRIM_CYLINDER` — vertical cylinder; `extent.n` = radius,
 *                         `extent.e` unused, `extent.alt` = half-height.
 *                         Cylinder axis is parallel to the altitude axis.
 *                         Rotation fields ignored.
 *   `SDF_PRIM_SPHERE`   — sphere; `extent.n` = radius, other channels
 *                         unused. Rotation fields ignored.
 *   `SDF_PRIM_PLANE`    — infinite horizontal plane at altitude
 *                         `center.alt`, normal pointing up. SDF is
 *                         `p.alt - center.alt` (above = positive).
 *                         `extent` and rotation fields unused.
 *
 * Negative extents are not validated; callers pass non-negative values.
 *
 * ## Rotation field
 *
 * `sdf_prim_t` carries `cos_yaw` / `sin_yaw` so that walls
 * placed by `scene_wall_at_cell()` (lib/terrain_sdf/scene.c) can sit at
 * arbitrary yaw in the N-E plane. The fields are precomputed at wall
 * generation time so the sphere tracer's `sdf_box()` body — called
 * ~216 k times per second under the obstacle ring (72 bins × ~10 steps
 * × ~30 prims × 10 Hz) — stays free of `sinf` / `cosf` calls. Per-call
 * cost is 4 muls + 2 adds + 2 subs for the rotation, vs ~4–5 % of a
 * 480 MHz Cortex-M7 core if the trig were evaluated on the hot path.
 *
 * Primitives where rotation has no geometric meaning (cylinder about
 * the altitude axis, sphere, horizontal plane) ignore the fields. The
 * memory cost is paid uniformly anyway because the scene array is a
 * fixed-stride `sdf_prim_t[]`.
 *
 * ## Origin-inside contract (pinned)
 *
 * `sdf_sphere_trace` returns the sentinel value `-1.f` when the ray
 * origin lies inside scene geometry — i.e. when
 * `scene_eval(origin) < 0`. Callers must map `-1.f` to "vehicle is
 * clipping geometry; report zero distance with quality marker", not to
 * "no hit found" (which is `max_t`). Mapping it to a clear reading when
 * the beam origin is already inside geometry would under-warn collision
 * prevention.
 *
 * ## API stability
 *
 * Signature changes require coordination with the companion viewer,
 * which compiles the same source. Adding new primitive types is
 * non-breaking; reordering or renaming enum values is breaking.
 *
 * ## References
 *
 *   [1] Hart, "Sphere tracing: a geometric method for the antialiased
 *       ray tracing of implicit surfaces", The Visual Computer 12(10),
 *       1996.
 *   [2] Quilez, "Distance functions",
 *       https://iquilezles.org/articles/distfunctions/, 2008.
 */

#ifndef PX4_SRC_LIB_TERRAIN_SDF_TERRAIN_SDF_H_
#define PX4_SRC_LIB_TERRAIN_SDF_TERRAIN_SDF_H_

#include <stdbool.h>

/** Maximum number of primitives the scene can hold. The list is static.
 *
 * Sized 64 to fit the lattice wall generator: with
 * `WALL_CELL_RADIUS_CELLS = 5` (121 cells) × `WALL_PRESENCE_P = 0.50`
 * × post-home-clearance survival ≈ 40–60 walls at a typical seed; 64
 * covers the high-tail seeds with headroom. Exceeding capacity causes
 * `sdf_scene_add` to silently drop the overflow (returns false); the
 * surviving walls still produce a valid scene, just with the unlucky
 * cells unrepresented.
 *
 * BSS cost at 64 prims × sizeof(sdf_prim_t) (~32 B with the
 * cos_yaw / sin_yaw rotation pair) ≈ 2 KB — negligible vs the 1 MB
 * SRAM on the supported boards.
 */
#define SDF_MAX_PRIMS 64

/**
 * 3D point / direction in altitude-up world frame.
 *
 *   n   — north metres
 *   e   — east  metres
 *   alt — altitude metres (positive up)
 */
typedef struct {
	float n;
	float e;
	float alt;
} sdf_vec3;

/** Primitive type discriminator. */
typedef enum {
	SDF_PRIM_BOX      = 0,
	SDF_PRIM_CYLINDER = 1,
	SDF_PRIM_SPHERE   = 2,
	SDF_PRIM_PLANE    = 3,
} sdf_prim_type_t;

/**
 * A scene primitive. See the file comment for per-type `extent`
 * semantics and the rotation-field contract.
 *
 * `cos_yaw` / `sin_yaw` carry a precomputed N-E-plane rotation. Boxes
 * use them to rotate the query point into the wall's local frame before
 * the axis-aligned box SDF. Cylinders / spheres / planes ignore them
 * (their geometry is invariant under N-E rotation). The fields are set
 * once at generation time by `scene_wall_at_cell()`; the sphere tracer
 * reads, never writes.
 */
typedef struct {
	sdf_prim_type_t type;
	sdf_vec3        center;
	sdf_vec3        extent;
	float           cos_yaw;
	float           sin_yaw;
} sdf_prim_t;

/**
 * Reset the scene to empty.
 *
 * Removes all previously added primitives. The heightfield SDF (which
 * is implicit, not added via `sdf_scene_add`) remains active. After
 * clearing, the scene contains only the heightfield, so a downward
 * beam returns the same answer the analytical `raycast()` would.
 */
void sdf_scene_clear(void);

/**
 * Enable/disable the procedural wall lattice.
 *
 * When enabled, `scene_eval()` (and therefore `sdf_sphere_trace()`)
 * queries walls lazily on demand via `scene_wall_at_cell()` for the
 * lattice cells local to each sample point — there is no materialized
 * wall list. When disabled (the default), no walls contribute. Gate
 * this on walls mode (`SIH_TERR_EN == 2`); the seed is read live from
 * `terrain_get_seed()` so changing it takes effect immediately.
 */
void sdf_walls_set_enabled(bool enabled);

/**
 * Append a primitive to the scene.
 *
 * @param p  Pointer to the primitive to copy in. The library takes a
 *           value copy; the caller's `sdf_prim_t` does not need to
 *           outlive this call.
 * @return   `true` on success, `false` if the scene is already at
 *           `SDF_MAX_PRIMS` capacity or `p == NULL`.
 */
bool sdf_scene_add(const sdf_prim_t *p);

/**
 * Cast a ray against the current scene and return the distance along
 * the ray to the first surface hit.
 *
 * The scene is the `min()`-union of all primitives added via
 * `sdf_scene_add` plus the implicit heightfield SDF wrapping
 * `terrain()`. The tracer is Hart 1996 sphere tracing with a minimum
 * step floor and stagnation detection (see implementation for the
 * exact constants).
 *
 * Returns:
 *   - `t` in `[0, max_t]` on hit — the distance along the ray at
 *     which `scene_eval(origin + t * dir_unit) < SDF_HIT_EPS`.
 *   - `max_t` when the ray exits the search range without hitting
 *     anything (the "miss" case).
 *   - `-1.f` (sentinel) when the origin itself is inside scene
 *     geometry, i.e. `scene_eval(origin) < 0`. See the origin-inside
 *     contract in the file comment. Callers must distinguish this
 *     from "miss" — `max_t` means clear ahead, `-1.f` means the
 *     vehicle is clipping geometry.
 *
 * @param origin    Ray origin (altitude-up).
 * @param dir_unit  Unit-length ray direction (altitude-up). Non-unit
 *                  directions scale the returned `t` by the inverse
 *                  of the magnitude and are not validated.
 * @param max_t     Maximum search distance along the ray [m]. Values
 *                  `<= 0.f` return `0.f`.
 * @return          Distance along the ray to the first hit in
 *                  `[0, max_t]`, or `max_t` on miss, or `-1.f` on
 *                  origin-inside.
 */
float sdf_sphere_trace(sdf_vec3 origin, sdf_vec3 dir_unit, float max_t);

#endif /* PX4_SRC_LIB_TERRAIN_SDF_TERRAIN_SDF_H_ */
