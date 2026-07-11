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
 * @file scene.h
 *
 * Procedural SDF wall lattice for SIH.
 *
 * Companion to `terrain_sdf.h`. Walls are derived from the shared
 * `SIH_TERR_SEED` via `hash3d(seed ^ WALL_HASH_SALT ^ cell_pack,
 * channel, 0)`. PX4 firmware and the companion viewer compile the same
 * source and produce identical primitive layouts — the same
 * bytes-must-match contract that already protects the integer hash
 * exported from `lib/terrain`. No MAVLink transport required for the
 * scene.
 *
 * ## Lattice walls
 *
 * Wall placement is a value-noise-style lattice: each cell
 * `(i_cell, j_cell)` in a fixed grid around home is independently
 * hashed, and that hash drives wall presence + dimensions. In walls
 * mode the `terrain()` heightfield is flat; the only ground geometry
 * above the flat floor is whatever this generator produces.
 *
 * ## Wall lattice geometry
 *
 * Each cell draws from six independent hash channels:
 *
 *   channel 0 (PRESENCE):  wall iff hash_to_unit_float < WALL_PRESENCE_P
 *   channel 1 (N_OFF):     N-offset of wall centre within cell, clamped
 *                          so the rotated wall's bounding box stays
 *                          inside its cell (best-effort; long walls at
 *                          adverse yaw can spill into adjacent cells)
 *   channel 2 (E_OFF):     E-offset within cell, same semantics
 *   channel 3 (LENGTH):    long-axis length, uniform on [WALL_MIN_LEN_M,
 *                          WALL_MAX_LEN_M)
 *   channel 4 (HEIGHT):    height, uniform on [WALL_HEIGHT_MIN_M,
 *                          WALL_HEIGHT_MAX_M)
 *   channel 5 (YAW):       yaw of the long axis in the N-E plane,
 *                          uniform on [0, π) — yaw+π is the same box
 *                          so the half-circle covers all orientations
 *
 * Walls are `sdf_box` primitives carrying precomputed `cos_yaw` /
 * `sin_yaw` so the sphere tracer can rotate the query point into the
 * wall's local frame without per-trace trig (see `terrain_sdf.h`).
 *
 * ## Home clearance
 *
 * Cells whose centre lies within `WALL_HOME_CLEARANCE_M` of home
 * produce no wall (`scene_wall_at_cell` returns `false`). With the
 * default `WALL_CELL_SPACING_M = 10` and `WALL_HOME_CLEARANCE_M = 15`,
 * the (0, 0) cell plus the eight 1-step neighbours are guaranteed
 * clear — vehicle home-spawn radius is unobstructed regardless of seed.
 *
 * ## API
 *
 *   scene_wall_at_cell(i, j, *out)  — returns true iff cell (i, j)
 *                                    hosts a wall; fills *out with the
 *                                    primitive on true.
 *
 * SIH iterates `(i, j) ∈ [-WALL_CELL_RADIUS_CELLS,
 * +WALL_CELL_RADIUS_CELLS]²` and adds each true-return primitive to
 * the SDF scene when `SIH_TERR_EN = 2` (walls mode). The default 5-cell radius =
 * 121 cells = ~40-60 walls at typical seed (presence × post-clearance
 * fraction). The actual wall count for a given seed can be inspected
 * with `Tools/scene_query`.
 */

#ifndef PX4_SRC_LIB_TERRAIN_SDF_SCENE_H_
#define PX4_SRC_LIB_TERRAIN_SDF_SCENE_H_

#include "terrain_sdf.h"

#include <stdbool.h>
#include <stdint.h>

/* Lattice geometry — fixed at compile time. */
#define WALL_CELL_SPACING_M     10.0f
#define WALL_PRESENCE_P          0.50f
#define WALL_CELL_RADIUS_CELLS   5

/* Wall dimensions. */
#define WALL_HOME_CLEARANCE_M   15.0f
#define WALL_MAX_LEN_M          15.0f
#define WALL_MIN_LEN_M           5.0f
#define WALL_THICKNESS_M         1.0f
#define WALL_HEIGHT_MIN_M       10.0f
#define WALL_HEIGHT_MAX_M       40.0f

/* Hash salt — keeps wall channels disjoint from any future primitive
 * type that picks its own salt. Bytes "WALL" as a little-endian
 * uint32. */
#define WALL_HASH_SALT          0x57414C4Cu

/* Channel indices — stable so seed → world mapping doesn't shift on
 * recompile. Reordering or repurposing a channel breaks the
 * shared-bytes contract with the companion viewer AND shifts every seeded fixture
 * in the test suite, so each channel id is load-bearing. */
#define WALL_CHANNEL_PRESENCE   0u
#define WALL_CHANNEL_N_OFF      1u
#define WALL_CHANNEL_E_OFF      2u
#define WALL_CHANNEL_LENGTH     3u
#define WALL_CHANNEL_HEIGHT     4u
#define WALL_CHANNEL_YAW        5u

/**
 * Cell-occupancy query: does lattice cell `(i_cell, j_cell)` host a
 * wall? Pure deterministic function of `(terrain_get_seed() ^
 * WALL_HASH_SALT, i_cell, j_cell)`.
 *
 * Returns `true` if a wall is present, and fills `*out` with the wall's
 * `sdf_prim_t` (centre / extent / precomputed cos_yaw / sin_yaw).
 * Returns `false` if the cell has no wall — either by the presence
 * channel landing above `WALL_PRESENCE_P`, or by the cell centre lying
 * within the home-clearance radius. `*out` is not modified on false.
 *
 * The cell centre in NED metres is
 * `(i_cell * WALL_CELL_SPACING_M, j_cell * WALL_CELL_SPACING_M)`. The
 * wall's centre is the cell centre offset by `(n_off, e_off)`, where
 * `n_off` / `e_off` are derived from channels 1 / 2 and clamped so the
 * wall's rotated bounding box stays inside its cell on a best-effort
 * basis. Long walls at adverse yaw may spill into adjacent cells; the
 * lattice tolerates this since clipping the offset to 0 for long walls
 * is still preferable to inter-cell collisions for short walls.
 */
bool scene_wall_at_cell(int32_t i_cell, int32_t j_cell, sdf_prim_t *out);

#endif /* PX4_SRC_LIB_TERRAIN_SDF_SCENE_H_ */
