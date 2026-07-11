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
 * To run this test only use: make tests TESTFILTER=terrain_sdf
 *
 * Wall-lattice (`scene.c`) tests. The companion `test_terrain_sdf.cpp`
 * exercises the sphere tracer and the analytic primitive SDFs; this file
 * exercises the procedural wall field that feeds them.
 *
 * Mirrors the structure of `lib/terrain`'s `test_terrain.cpp` because the
 * wall lattice is the same kind of object the terrain heightfield is: a
 * pure, seeded, on-demand field with no stored state.
 *
 * Test plan:
 *   (a) walls disabled (the default) contribute nothing to the scene
 *   (b) no-stored-list contract — with the generic primitive list empty,
 *       enabling walls still produces hits, proving walls are queried on
 *       demand and never materialised into the SDF_MAX_PRIMS array
 *   (c) bit-exact determinism — same seed -> byte-identical primitive
 *   (d) a reconfigure that returns to the same seed reproduces the layout
 *   (e) different seeds produce a different layout
 *   (f) every wall is one of two perpendicular axis-aligned orientations
 *       (cos_yaw = 1, sin_yaw = 0; one extent is the half-thickness, the
 *       other the half-length) — the post-rewrite geometry contract
 *   (g) home clearance — no wall within WALL_HOME_CLEARANCE_M of home,
 *       across several seeds
 *   (h) the on-demand field reports origin-inside (sentinel -1) when the
 *       vehicle sits inside a wall, and a finite hit when aimed at one
 *   (i) per-call cost budget for scene_wall_at_cell()
 *
 * Each test sets the seed up front via terrain_set_params() (the wall
 * generator reads terrain_get_seed() live) and toggles
 * sdf_walls_set_enabled() / sdf_scene_clear() because both libraries hold
 * static state across tests and gtest does not guarantee test ordering.
 */

#include <gtest/gtest.h>

#include <lib/terrain_sdf/terrain_sdf.h>
#include <lib/terrain_sdf/scene.h>
#include <lib/terrain/terrain.h>

#include <chrono>
#include <cmath>
#include <cstring>

namespace
{

constexpr float kHalfThick = WALL_THICKNESS_M * 0.5f;   /* 0.5 m */
constexpr float kHalfLenMin = WALL_MIN_LEN_M * 0.5f;    /* 2.5 m */
constexpr float kHalfLenMax = WALL_MAX_LEN_M * 0.5f;    /* 7.5 m */

/* Configure a flat heightfield under the given seed. amp = 0 removes the
 * terrain heightfield so only the wall field contributes geometry; the
 * seed still flows into the wall generator via terrain_get_seed(). */
void configure_seed(int seed)
{
	terrain_set_params(0.f, 200.f, seed, 0.f, 0.f, 0.f);
}

/* Scan the lattice for the first present wall and return it. Returns
 * false if the seed somehow yields no walls in range (never happens for
 * the seeds used here, but keep the tests honest). */
bool first_wall(sdf_prim_t *out, int32_t *ci = nullptr, int32_t *cj = nullptr)
{
	for (int32_t i = -WALL_CELL_RADIUS_CELLS; i <= WALL_CELL_RADIUS_CELLS; ++i) {
		for (int32_t j = -WALL_CELL_RADIUS_CELLS; j <= WALL_CELL_RADIUS_CELLS; ++j) {
			if (scene_wall_at_cell(i, j, out)) {
				if (ci) { *ci = i; }
				if (cj) { *cj = j; }
				return true;
			}
		}
	}

	return false;
}

} // namespace

/* --------------------------------------------------------------------------
 * (a) Disabled by default — walls contribute nothing.
 * -------------------------------------------------------------------------- */

TEST(TestScene, DisabledMeansNoWalls)
{
	configure_seed(3);
	sdf_scene_clear();
	sdf_walls_set_enabled(false);

	/* A long horizontal ray across the lattice at a wall-typical altitude.
	 * With walls off and a flat heightfield, nothing is in the way. */
	const sdf_vec3 origin = { -60.f, 0.f, 8.f };
	const sdf_vec3 dir    = { 1.f, 0.f, 0.f };
	const float    max_t  = 120.f;

	EXPECT_FLOAT_EQ(sdf_sphere_trace(origin, dir, max_t), max_t);
}

/* --------------------------------------------------------------------------
 * (b) No-stored-list contract — walls are an on-demand field.
 *
 * The generic primitive list is left empty (no sdf_scene_add). If enabling
 * walls makes a previously-clear ray hit something, that geometry can only
 * have come from the lazy scene_wall_at_cell() query inside scene_eval(),
 * never from the SDF_MAX_PRIMS array. This is the wall analogue of
 * terrain's LookupTableSizeIsZero contract: cheap, seeded, no precompute.
 * -------------------------------------------------------------------------- */

TEST(TestScene, EnabledProducesHitsWithEmptyList)
{
	configure_seed(3);
	sdf_scene_clear();   /* generic list empty — no sdf_scene_add() calls */

	sdf_prim_t wall;
	ASSERT_TRUE(first_wall(&wall));

	/* Aim a ray straight at the wall centre from 30 m out along -N, at the
	 * wall's own centre E / altitude so the beam stays within its E and
	 * altitude spans. */
	const float   approach = 30.f;
	const sdf_vec3 origin  = { wall.center.n - approach, wall.center.e, wall.center.alt };
	const sdf_vec3 dir     = { 1.f, 0.f, 0.f };
	const float    max_t   = approach + 5.f;

	sdf_walls_set_enabled(false);
	const float t_off = sdf_sphere_trace(origin, dir, max_t);

	sdf_walls_set_enabled(true);
	const float t_on = sdf_sphere_trace(origin, dir, max_t);

	EXPECT_FLOAT_EQ(t_off, max_t) << "walls off: nothing in an empty scene";
	EXPECT_LT(t_on, max_t) << "walls on: the lazy field put a surface in the path";
	EXPECT_GT(t_on, 0.f);
}

/* --------------------------------------------------------------------------
 * (c) Bit-exact determinism for the same seed.
 * -------------------------------------------------------------------------- */

TEST(TestScene, IsBitExactRepeatableForSameSeed)
{
	configure_seed(3);

	for (int32_t i = -WALL_CELL_RADIUS_CELLS; i <= WALL_CELL_RADIUS_CELLS; ++i) {
		for (int32_t j = -WALL_CELL_RADIUS_CELLS; j <= WALL_CELL_RADIUS_CELLS; ++j) {
			sdf_prim_t a;
			sdf_prim_t b;
			const bool pa = scene_wall_at_cell(i, j, &a);
			const bool pb = scene_wall_at_cell(i, j, &b);

			EXPECT_EQ(pa, pb);

			if (pa) {
				EXPECT_EQ(0, std::memcmp(&a, &b, sizeof(sdf_prim_t)))
						<< "cell (" << i << ", " << j << ") not bit-stable";
			}
		}
	}
}

/* --------------------------------------------------------------------------
 * (d) Returning to a seed reproduces its layout (no perturbation from an
 *     intervening reconfigure).
 * -------------------------------------------------------------------------- */

TEST(TestScene, SameSeedReproducibleAcrossReconfigure)
{
	configure_seed(3);
	sdf_prim_t first;
	int32_t ci = 0;
	int32_t cj = 0;
	ASSERT_TRUE(first_wall(&first, &ci, &cj));

	/* Different seed in between, then back. */
	configure_seed(7);
	configure_seed(3);

	sdf_prim_t again;
	ASSERT_TRUE(scene_wall_at_cell(ci, cj, &again));
	EXPECT_EQ(0, std::memcmp(&first, &again, sizeof(sdf_prim_t)));
}

/* --------------------------------------------------------------------------
 * (e) Different seeds produce a different layout.
 * -------------------------------------------------------------------------- */

TEST(TestScene, DifferentSeedsDiffer)
{
	int presence_diffs = 0;
	int geometry_diffs = 0;

	for (int32_t i = -WALL_CELL_RADIUS_CELLS; i <= WALL_CELL_RADIUS_CELLS; ++i) {
		for (int32_t j = -WALL_CELL_RADIUS_CELLS; j <= WALL_CELL_RADIUS_CELLS; ++j) {
			configure_seed(3);
			sdf_prim_t a;
			const bool pa = scene_wall_at_cell(i, j, &a);

			configure_seed(7);
			sdf_prim_t b;
			const bool pb = scene_wall_at_cell(i, j, &b);

			if (pa != pb) {
				++presence_diffs;

			} else if (pa && std::memcmp(&a, &b, sizeof(sdf_prim_t)) != 0) {
				++geometry_diffs;
			}
		}
	}

	EXPECT_GT(presence_diffs + geometry_diffs, 0)
			<< "seeds 3 and 7 produced an identical lattice";
}

/* --------------------------------------------------------------------------
 * (f) Two perpendicular axis-aligned orientations.
 *
 * The rewrite dropped continuous yaw: every wall is axis-aligned
 * (cos_yaw = 1, sin_yaw = 0). Orientation is encoded purely by which
 * planar extent carries the half-length — the other is the fixed
 * half-thickness. This is what lets the tracer skip rotation and the
 * field stay trig-free (and bit-exact across platforms).
 * -------------------------------------------------------------------------- */

TEST(TestScene, WallsAreAxisAlignedTwoOrientations)
{
	for (int seed : { 3, 6, 7, 42 }) {
		configure_seed(seed);

		for (int32_t i = -WALL_CELL_RADIUS_CELLS; i <= WALL_CELL_RADIUS_CELLS; ++i) {
			for (int32_t j = -WALL_CELL_RADIUS_CELLS; j <= WALL_CELL_RADIUS_CELLS; ++j) {
				sdf_prim_t w;

				if (!scene_wall_at_cell(i, j, &w)) {
					continue;
				}

				EXPECT_EQ(SDF_PRIM_BOX, w.type);
				EXPECT_FLOAT_EQ(1.f, w.cos_yaw);
				EXPECT_FLOAT_EQ(0.f, w.sin_yaw);

				/* One planar extent is the half-thickness; the other
				 * is the half-length. half_thick (0.5) is always
				 * strictly less than half_len (>= 2.5), so the smaller
				 * planar extent is the thin one — pick it with '<' to
				 * avoid an exact float compare. */
				const bool  n_thin   = w.extent.n < w.extent.e;
				const float thin     = n_thin ? w.extent.n : w.extent.e;
				const float half_len = n_thin ? w.extent.e : w.extent.n;

				EXPECT_FLOAT_EQ(kHalfThick, thin)
						<< "cell (" << i << ", " << j << "): thin axis not half-thickness";
				EXPECT_GE(half_len, kHalfLenMin);
				EXPECT_LE(half_len, kHalfLenMax);

				/* Height half-extent is in the configured band. */
				EXPECT_GE(w.extent.alt, WALL_HEIGHT_MIN_M * 0.5f);
				EXPECT_LE(w.extent.alt, WALL_HEIGHT_MAX_M * 0.5f);
			}
		}
	}
}

/* --------------------------------------------------------------------------
 * (g) Home clearance — spawn region is unobstructed for any seed.
 * -------------------------------------------------------------------------- */

TEST(TestScene, HomeClearanceKeepsSpawnClear)
{
	for (int seed : { 3, 6, 7, 42, 1234 }) {
		configure_seed(seed);

		for (int32_t i = -WALL_CELL_RADIUS_CELLS; i <= WALL_CELL_RADIUS_CELLS; ++i) {
			for (int32_t j = -WALL_CELL_RADIUS_CELLS; j <= WALL_CELL_RADIUS_CELLS; ++j) {
				const float cn = (float)i * WALL_CELL_SPACING_M;
				const float ce = (float)j * WALL_CELL_SPACING_M;

				if ((cn * cn) + (ce * ce)
				    >= (WALL_HOME_CLEARANCE_M * WALL_HOME_CLEARANCE_M)) {
					continue;
				}

				sdf_prim_t w;
				EXPECT_FALSE(scene_wall_at_cell(i, j, &w))
						<< "seed " << seed << " placed a wall in the home-clearance cell ("
						<< i << ", " << j << ")";
			}
		}
	}
}

/* --------------------------------------------------------------------------
 * (h) On-demand field correctness through the tracer: origin-inside
 *     sentinel and a finite hit when aimed at a wall.
 * -------------------------------------------------------------------------- */

TEST(TestScene, InsideAWallReturnsOriginInsideSentinel)
{
	configure_seed(3);
	sdf_scene_clear();
	sdf_walls_set_enabled(true);

	sdf_prim_t wall;
	ASSERT_TRUE(first_wall(&wall));

	/* Origin at the wall centre — unambiguously inside the box. The tracer
	 * must return the -1 origin-inside sentinel, not max_t (miss). */
	const sdf_vec3 origin = { wall.center.n, wall.center.e, wall.center.alt };
	const sdf_vec3 dir    = { 1.f, 0.f, 0.f };

	EXPECT_FLOAT_EQ(sdf_sphere_trace(origin, dir, 50.f), -1.f);
}

/* --------------------------------------------------------------------------
 * (i) Per-call cost budget for scene_wall_at_cell().
 *
 * Mirrors terrain's ThousandCallsUnderTenMs. The wall field is queried on
 * the obstacle-ring hot path; a regression that reintroduced trig or a
 * stored-list scan would show up here. Full-lattice sweeps (121 cells) ×
 * repeats = ~12 k calls, comfortably under budget on host.
 * -------------------------------------------------------------------------- */

TEST(TestScene, ThousandCallsUnderTenMs)
{
	configure_seed(3);

	const auto t0 = std::chrono::steady_clock::now();

	volatile int sink = 0;
	const int repeats = 100;   /* 100 * 121 = 12 100 cell queries */

	for (int r = 0; r < repeats; ++r) {
		for (int32_t i = -WALL_CELL_RADIUS_CELLS; i <= WALL_CELL_RADIUS_CELLS; ++i) {
			for (int32_t j = -WALL_CELL_RADIUS_CELLS; j <= WALL_CELL_RADIUS_CELLS; ++j) {
				sdf_prim_t w;
				sink += scene_wall_at_cell(i, j, &w) ? 1 : 0;
			}
		}
	}

	const auto t1 = std::chrono::steady_clock::now();
	const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

	(void)sink;
	EXPECT_LT(us, 10000) << "12 100 scene_wall_at_cell() calls took " << us
			     << " us (budget 10 000 us)";
}
