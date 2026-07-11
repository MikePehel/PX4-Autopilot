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
 * @file scene.c
 *
 * Procedural SDF wall lattice for SIH. See scene.h for the geometry
 * contract.
 *
 * This reinterprets the terrain noise lattice: each cell hash means
 * "is there a wall here, and what shape" instead of summing into an
 * elevation. Same hash primitive (`hash3d` from `lib/terrain`), same
 * bytes-must-match guarantee with the companion viewer; different
 * meaning.
 *
 * Constraints (matching `lib/terrain` and `lib/terrain_sdf`):
 *   - Pure C, only <math.h> + <stdint.h>.
 *   - No heap, no dynamic allocation.
 *   - Bit-exact integer hash across ARM Cortex-M7 / x86_64 / wasm32.
 */

#include "scene.h"

#include "../terrain/terrain.h"   /* hash3d, hash_to_unit_float, terrain_get_seed */

#include <math.h>
#include <stddef.h>

/* Local π — terrain.c carries the same trick to stay portable across
 * libc <math.h> variants that gate M_PI behind feature macros. */
#define SCENE_PI   3.14159265358979323846f

/* Pack a signed cell coordinate into uint32 via two's-complement
 * reinterpretation. Bit-exact across every platform PX4 supports
 * (every compiler that respects 32-bit two's-complement int). The
 * lattice key for cell (i, j) is hash3d(seed_mixed, i_u, j_u) where
 * (i_u, j_u) are the packed coordinates. Channel index then mixes
 * back in via a second hash layer. */
static inline uint32_t pack_cell(int32_t v)
{
	return (uint32_t)v;
}

/* Cell hash: derive one uint32 of entropy for `(cell_i, cell_j, channel)`
 * under the configured seed. Two hash layers so that (cell, channel)
 * fully decorrelate even when cells are adjacent and channels small.
 * The seed mix XORs in WALL_HASH_SALT so future primitive types can
 * pick their own salts without colliding. */
static inline uint32_t cell_channel_hash(int32_t i_cell, int32_t j_cell, uint32_t channel)
{
	const uint32_t seed_mixed = (uint32_t)terrain_get_seed() ^ WALL_HASH_SALT;
	const uint32_t cell_hash  = hash3d(seed_mixed, pack_cell(i_cell), pack_cell(j_cell));
	return hash3d(cell_hash, channel, 0u);
}

bool scene_wall_at_cell(int32_t i_cell, int32_t j_cell, sdf_prim_t *out)
{
	if (out == NULL) {
		return false;
	}

	/* Channel 0 — presence. Skip cells whose hash lands above the
	 * threshold. Done first so the cheap rejection short-circuits
	 * the remaining channels. */
	const float u_presence = hash_to_unit_float(cell_channel_hash(i_cell, j_cell, WALL_CHANNEL_PRESENCE));

	if (u_presence >= WALL_PRESENCE_P) {
		return false;
	}

	/* Home clearance — cells whose centre is within
	 * WALL_HOME_CLEARANCE_M of (0, 0) host no wall, preserving the
	 * vehicle spawn region regardless of seed. With the default
	 * spacing (10 m) and clearance (15 m), this clears the (0, 0)
	 * cell plus the eight 1-step neighbours (the 8-conn ring out
	 * to |cell| = sqrt(2) * spacing ≈ 14.14 m). The r² compare
	 * avoids a sqrtf. */
	const float cell_n = (float)i_cell * WALL_CELL_SPACING_M;
	const float cell_e = (float)j_cell * WALL_CELL_SPACING_M;

	if ((cell_n * cell_n) + (cell_e * cell_e)
	    < (WALL_HOME_CLEARANCE_M * WALL_HOME_CLEARANCE_M)) {
		return false;
	}

	/* Channels 3, 4 — length, height. Pulled before offsets so the
	 * offset-clamp range can be computed from the rotated bbox. */
	const float u_length = hash_to_unit_float(cell_channel_hash(i_cell, j_cell, WALL_CHANNEL_LENGTH));
	const float u_height = hash_to_unit_float(cell_channel_hash(i_cell, j_cell, WALL_CHANNEL_HEIGHT));
	const float length   = WALL_MIN_LEN_M    + (WALL_MAX_LEN_M    - WALL_MIN_LEN_M)    * u_length;
	const float height   = WALL_HEIGHT_MIN_M + (WALL_HEIGHT_MAX_M - WALL_HEIGHT_MIN_M) * u_height;

	/* Channel 5 — orientation. Walls take one of two perpendicular,
	 * axis-aligned headings: long axis along N, or along E (a single
	 * hash bit picks which). That makes every wall an axis-aligned box
	 * (cos_yaw = 1, sin_yaw = 0), so the per-step box SDF takes its
	 * zero-cost no-rotation path and scene_wall_at_cell() needs no trig
	 * at all — the wall lattice is then as cheap to evaluate on demand
	 * as the terrain heightfield, with no precompute or stored list. */
	const float u_yaw     = hash_to_unit_float(cell_channel_hash(i_cell, j_cell, WALL_CHANNEL_YAW));
	const bool  e_aligned = (u_yaw >= 0.5f);

	/* Channels 1, 2 — N / E offset within cell. Best-effort
	 * clamping: compute the rotated-bbox half-diagonal for this
	 * wall (0.5 * sqrt(length² + thickness²)), then allow the
	 * centre to wander within the cell up to the slack that leaves
	 * after subtracting the rotated half. When the rotated half
	 * exceeds the cell half-extent, slack collapses to 0 and the
	 * wall is pinned at the cell centre — long walls in a narrow
	 * cell stay centred rather than poking aggressively into
	 * neighbours. Short walls retain full lateral freedom.
	 *
	 * This does not strictly prevent inter-cell overlap on the
	 * boundary between two adjacent occupied cells with long walls
	 * at adverse yaw; that's accepted per the plan. The contract
	 * is "best-effort, never break home clearance," not "exact
	 * bbox containment." */
	const float rot_half  = 0.5f * sqrtf(length * length
					     + WALL_THICKNESS_M * WALL_THICKNESS_M);
	const float cell_half = 0.5f * WALL_CELL_SPACING_M;
	float       slack     = cell_half - rot_half;

	if (slack < 0.f) {
		slack = 0.f;
	}

	const float u_n_off = hash_to_unit_float(cell_channel_hash(i_cell, j_cell, WALL_CHANNEL_N_OFF));
	const float u_e_off = hash_to_unit_float(cell_channel_hash(i_cell, j_cell, WALL_CHANNEL_E_OFF));
	const float n_off   = (u_n_off - 0.5f) * (2.f * slack);
	const float e_off   = (u_e_off - 0.5f) * (2.f * slack);

	/* Assemble the primitive. The box is axis-aligned (cos_yaw = 1,
	 * sin_yaw = 0); orientation is encoded purely by which axis carries
	 * the long half-length. N-aligned: extent.n = half-length,
	 * extent.e = half-thickness. E-aligned: the two are swapped. The
	 * sphere tracer's sdf_box() sees cos_yaw = 1 / sin_yaw = 0 and skips
	 * the rotation entirely — see lib/terrain_sdf. */
	const float half_len   = length * 0.5f;
	const float half_thick = WALL_THICKNESS_M * 0.5f;

	out->type       = SDF_PRIM_BOX;
	out->center.n   = cell_n + n_off;
	out->center.e   = cell_e + e_off;
	out->center.alt = height * 0.5f;
	out->extent.n   = e_aligned ? half_thick : half_len;
	out->extent.e   = e_aligned ? half_len   : half_thick;
	out->extent.alt = height * 0.5f;
	out->cos_yaw    = 1.f;
	out->sin_yaw    = 0.f;
	return true;
}
