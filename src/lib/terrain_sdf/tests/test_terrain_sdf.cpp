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
 * Test plan:
 *   (a) sphere_trace hits an axis-aligned box at the expected face distance
 *       (cos_yaw=1, sin_yaw=0 — no rotation)
 *   (b) sphere_trace hits a vertical cylinder at the expected face distance
 *   (c) sphere_trace returns max_t when the scene has nothing to hit
 *   (d) sphere_trace returns the -1.f sentinel when the origin is inside
 *       a box (origin-inside contract)
 *   (e) heightfield-only beam returns the analytical-equivalent distance
 *   (f) sphere_trace hits a yaw-rotated box on its true (rotated) face,
 *       not the axis-aligned bounding box — exercises the cos_yaw / sin_yaw
 *       rotation path in prim_sdf_box
 *
 * Each test calls terrain_set_params() + sdf_scene_clear() up front
 * because both libraries hold static state across tests and gtest
 * does not guarantee test ordering.
 */

#include <gtest/gtest.h>

#include <lib/terrain_sdf/terrain_sdf.h>
#include <lib/terrain/terrain.h>

#include <cmath>

namespace
{

void configure_flat_terrain()
{
	/* Force terrain(N, E) = 0 everywhere by setting amp = 0. Removes
	 * the fBm heightfield from interfering with primitive-hit tests. */
	terrain_set_params(0.f, 200.f, 0, 0.f, 0.f, 0.f);
}

} // namespace

/* --------------------------------------------------------------------------
 * (a) Axis-aligned box hit
 * -------------------------------------------------------------------------- */

TEST(TestTerrainSdf, SphereTraceHitsAxisAlignedBox)
{
	configure_flat_terrain();
	sdf_scene_clear();

	/* Box from N=[15, 25], E=[-5, 5], alt=[0, 5]. Axis-aligned →
	 * cos_yaw=1, sin_yaw=0 (identity rotation). */
	const sdf_prim_t box = {
		SDF_PRIM_BOX,
		{ 20.f, 0.f, 2.5f },     // center
		{ 5.f,  5.f, 2.5f },     // half-extents in local frame
		1.f,                     // cos_yaw — identity
		0.f,                     // sin_yaw
	};
	ASSERT_TRUE(sdf_scene_add(&box));

	/* Ray due north at altitude 2.5 m (inside the box's vertical span).
	 * Expected hit on the box's near face at N = 15.0. */
	const sdf_vec3 origin = { 0.f, 0.f, 2.5f };
	const sdf_vec3 dir    = { 1.f, 0.f, 0.f };
	const float    max_t  = 50.f;

	float t = sdf_sphere_trace(origin, dir, max_t);

	EXPECT_NEAR(t, 15.f, 0.05f);
	EXPECT_LE(t, 15.f);   /* sphere tracer converges from the lit side */
}

/* --------------------------------------------------------------------------
 * (b) Vertical cylinder hit
 * -------------------------------------------------------------------------- */

TEST(TestTerrainSdf, SphereTraceHitsVerticalCylinder)
{
	configure_flat_terrain();
	sdf_scene_clear();

	/* Cylinder: centre (0, 20, 5), radius 2, half-height 5. Cylinders
	 * are invariant under N-E rotation, so cos_yaw / sin_yaw are
	 * ignored — set to (1, 0) anyway for cleanliness. */
	const sdf_prim_t cyl = {
		SDF_PRIM_CYLINDER,
		{ 0.f, 20.f, 5.f },          // center
		{ 2.f, 0.f, 5.f },           // (radius, unused, half-height)
		1.f,
		0.f,
	};
	ASSERT_TRUE(sdf_scene_add(&cyl));

	/* Ray due east at altitude 5 m, hitting the cylinder mid-height.
	 * Expected hit on the west surface at E = 18.0. */
	const sdf_vec3 origin = { 0.f, 0.f, 5.f };
	const sdf_vec3 dir    = { 0.f, 1.f, 0.f };
	const float    max_t  = 50.f;

	float t = sdf_sphere_trace(origin, dir, max_t);

	EXPECT_NEAR(t, 18.f, 0.05f);
	EXPECT_LE(t, 18.f);
}

/* --------------------------------------------------------------------------
 * (c) Miss returns max_t
 * -------------------------------------------------------------------------- */

TEST(TestTerrainSdf, SphereTraceReturnsMaxTOnMiss)
{
	configure_flat_terrain();
	sdf_scene_clear();

	/* No primitives. Ray runs horizontally at altitude 10 m — flat
	 * heightfield is at altitude 0, so the ray stays a constant 10 m
	 * above the surface and never hits anything. */
	const sdf_vec3 origin = { 0.f, 0.f, 10.f };
	const sdf_vec3 dir    = { 1.f, 0.f, 0.f };
	const float    max_t  = 50.f;

	float t = sdf_sphere_trace(origin, dir, max_t);

	EXPECT_FLOAT_EQ(t, max_t);
}

/* --------------------------------------------------------------------------
 * (d) Origin-inside contract: returns -1.f sentinel
 * -------------------------------------------------------------------------- */

TEST(TestTerrainSdf, SphereTraceReturnsSentinelWhenOriginInsideBox)
{
	configure_flat_terrain();
	sdf_scene_clear();

	const sdf_prim_t box = {
		SDF_PRIM_BOX,
		{ 0.f, 0.f, 0.f },
		{ 5.f, 5.f, 5.f },
		1.f,
		0.f,
	};
	ASSERT_TRUE(sdf_scene_add(&box));

	const sdf_vec3 origin = { 0.f, 0.f, 3.f };
	const sdf_vec3 dir    = { 1.f, 0.f, 0.f };

	float t = sdf_sphere_trace(origin, dir, 50.f);

	EXPECT_FLOAT_EQ(t, -1.f);
}

/* --------------------------------------------------------------------------
 * (e) Heightfield-only beam matches analytical formula
 * -------------------------------------------------------------------------- */

TEST(TestTerrainSdf, DownwardBeamOverFlatGroundMatchesAnalytical)
{
	configure_flat_terrain();
	sdf_scene_clear();

	const sdf_vec3 origin = { 0.f, 0.f, 10.f };
	const sdf_vec3 dir    = { 0.f, 0.f, -1.f };
	const float    max_t  = 50.f;

	float t = sdf_sphere_trace(origin, dir, max_t);

	EXPECT_NEAR(t, 10.f, 0.05f);
}

/* --------------------------------------------------------------------------
 * (f) Yaw-rotated box — cos_yaw / sin_yaw path
 *
 * Place a thin slab (10 m long along its local-N axis, 1 m thick along
 * its local-E axis) centred 20 m east of home, but rotated 45° in the
 * N-E plane. The long axis now runs SW-NE; from home (origin) a +E ray
 * at the slab's mid-height must hit the slab's rotated face, not the
 * unrotated axis-aligned bounding-box face.
 *
 * For yaw = +45°, the slab's local axes are
 *   local_n_world = (cos 45, sin 45) = (+0.707, +0.707)
 *   local_e_world = (-sin 45, cos 45) = (-0.707, +0.707)
 * The slab's two long faces (perpendicular to local_e_world) sit at
 * (E - 0.707·0.5, N - 0.707·0.5) and (E + 0.707·0.5, N + 0.707·0.5)
 * offsets from the centre. The east-ward face the +E ray crosses
 * first depends on where the ray intersects the rotated box surface;
 * approximate analytically: along a ray (0, E, 5) east-bound, the
 * slab's centre is at E=20; the closest face along that ray is at
 *   E = 20 - 0.5·thickness/|sin 45| ≈ 20 - 0.707 ≈ 19.29
 * provided the ray actually crosses the rotated face (which it does at
 * the centre line N=0). Hit window ≈ 19.0 to 19.5.
 * -------------------------------------------------------------------------- */

TEST(TestTerrainSdf, SphereTraceHitsYawRotatedBoxOnRotatedFace)
{
	configure_flat_terrain();
	sdf_scene_clear();

	const float yaw     = static_cast<float>(M_PI / 4.0); /* 45° */
	const float cos_yaw = std::cos(yaw);
	const float sin_yaw = std::sin(yaw);

	/* Slab: long 10 m, thick 1 m, tall 10 m. Centre 20 m east of home
	 * at mid-height (alt = 5). */
	const sdf_prim_t slab = {
		SDF_PRIM_BOX,
		{ 0.f, 20.f, 5.f },   // center
		{ 5.f, 0.5f, 5.f },   // half-extents in LOCAL (rotated) frame
		cos_yaw,
		sin_yaw,
	};
	ASSERT_TRUE(sdf_scene_add(&slab));

	const sdf_vec3 origin = { 0.f, 0.f, 5.f };
	const sdf_vec3 dir    = { 0.f, 1.f, 0.f };

	const float t = sdf_sphere_trace(origin, dir, 50.f);

	/* Without rotation the hit would be at E = 19.5 (the box's
	 * +E face). With 45° rotation the closest face along the +E ray
	 * sits at E ≈ 20 - 0.5/sin(45°) ≈ 19.29. The window below
	 * accommodates both the analytic answer and the sphere tracer's
	 * HIT_EPS band. */
	EXPECT_GT(t, 19.0f);
	EXPECT_LT(t, 19.5f);
}
