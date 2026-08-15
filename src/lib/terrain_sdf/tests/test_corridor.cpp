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
 * Exact ray/box intersection, which is what corridor mode rests on.
 *
 * Sphere marching steps by the distance to the nearest surface in ANY
 * direction, so between two walls the step collapses to the side clearance
 * regardless of how far the clear air ahead runs. Exhaust SDF_MAX_ITER and
 * the tracer returns max_t, and max_t is the same value SIH publishes to
 * mean "no obstacle in range". An exhausted ray and a clear ray become
 * indistinguishable to collision prevention, which is a silent failure:
 * nothing throws, the vehicle simply does not brake.
 *
 * These tests pin the property that removes that failure. Boxes are solved
 * in closed form, so the answer does not depend on how many steps were
 * available.
 *
 * Test plan:
 *   (a) a ray stops at a box's near FACE, not its centreline
 *   (b) a ray through a gap between two boxes reaches what lies beyond,
 *       which is the doorway case and the one marching gets wrong last
 *   (c) a ray parallel to a box and clear of it is not deflected by it
 *   (d) a box behind the origin is never reported
 *   (e) the answer is independent of corridor width, which is the whole
 *       claim: marching is not, exact intersection is
 *   (f) yawed boxes intersect correctly, since the wall lattice uses yaw
 */

#include <gtest/gtest.h>

#include <cmath>

#include "../terrain_sdf.h"
#include "../../terrain/terrain.h"

namespace
{

/* Flat, featureless ground far below, so only the boxes can be hit. */
void flat_world()
{
	terrain_set_params(0.f, 100.f, 0, 0.f, 0.f, 0.f);
	sdf_walls_set_enabled(false);
	sdf_scene_clear();
}

sdf_prim_t box(float n, float e, float alt, float hn, float he, float halt,
	       float cos_yaw = 1.f, float sin_yaw = 0.f)
{
	sdf_prim_t p{};
	p.type    = SDF_PRIM_BOX;
	p.center  = sdf_vec3{n, e, alt};
	p.extent  = sdf_vec3{hn, he, halt};
	p.cos_yaw = cos_yaw;
	p.sin_yaw = sin_yaw;
	return p;
}

const sdf_vec3 kNorth{1.f, 0.f, 0.f};
const sdf_vec3 kSouth{-1.f, 0.f, 0.f};

} // namespace

/* (a) a ray stops at the near face */
TEST(CorridorRay, StopsAtNearFaceNotCentre)
{
	flat_world();
	/* Wall centred at north 20, 1 m thick: near face at 19.5. */
	sdf_prim_t w = box(20.f, 0.f, 4.f, 0.5f, 12.f, 4.f);
	ASSERT_TRUE(sdf_scene_add(&w));

	EXPECT_NEAR(sdf_sphere_trace(sdf_vec3{0.f, 0.f, 2.f}, kNorth, 100.f), 19.5f, 1e-2f);
}

/* (b) the doorway case: a gap lets the ray through to what is beyond */
TEST(CorridorRay, PassesThroughADoorway)
{
	flat_world();
	/* Cross wall at north 15 split around a centred 8 m doorway. */
	sdf_prim_t jamb_w = box(15.f, -8.f, 4.f, 0.5f, 4.f, 4.f);
	sdf_prim_t jamb_e = box(15.f,  8.f, 4.f, 0.5f, 4.f, 4.f);
	/* End cap beyond it. */
	sdf_prim_t cap = box(45.f, 0.f, 4.f, 0.5f, 12.f, 4.f);
	ASSERT_TRUE(sdf_scene_add(&jamb_w));
	ASSERT_TRUE(sdf_scene_add(&jamb_e));
	ASSERT_TRUE(sdf_scene_add(&cap));

	/* Up the centreline the doorway is open, so the end cap is the hit. */
	EXPECT_NEAR(sdf_sphere_trace(sdf_vec3{0.f, 0.f, 2.f}, kNorth, 100.f), 44.5f, 1e-2f);

	/* Offset into a jamb and the same ray stops at the cross wall. */
	EXPECT_NEAR(sdf_sphere_trace(sdf_vec3{0.f, 8.f, 2.f}, kNorth, 100.f), 14.5f, 1e-2f);
}

/* (c) a box beside the ray does not deflect it */
TEST(CorridorRay, SideWallDoesNotBlockForwardTravel)
{
	flat_world();
	/* Long side wall 12 m to the east, parallel to the ray. */
	sdf_prim_t side = box(0.f, 12.5f, 4.f, 45.f, 0.5f, 4.f);
	sdf_prim_t cap  = box(45.f, 0.f, 4.f, 0.5f, 12.f, 4.f);
	ASSERT_TRUE(sdf_scene_add(&side));
	ASSERT_TRUE(sdf_scene_add(&cap));

	EXPECT_NEAR(sdf_sphere_trace(sdf_vec3{0.f, 0.f, 2.f}, kNorth, 100.f), 44.5f, 1e-2f);
}

/* (d) geometry behind the origin is never reported */
TEST(CorridorRay, IgnoresBoxesBehindTheRay)
{
	flat_world();
	sdf_prim_t behind = box(-20.f, 0.f, 4.f, 0.5f, 12.f, 4.f);
	ASSERT_TRUE(sdf_scene_add(&behind));

	/* Nothing ahead, so this is a clean miss out to max_t. */
	EXPECT_NEAR(sdf_sphere_trace(sdf_vec3{0.f, 0.f, 2.f}, kNorth, 60.f), 60.f, 1e-2f);
	/* Turned round it is found at its near face. */
	EXPECT_NEAR(sdf_sphere_trace(sdf_vec3{0.f, 0.f, 2.f}, kSouth, 60.f), 19.5f, 1e-2f);
}

/*
 * (e) the reason this exists. A marcher's step length is bounded by the side
 * clearance, so a narrow corridor costs more iterations than a wide one and a
 * long enough narrow corridor exhausts the budget and reports open air.
 * Exact intersection returns the same distance whatever the width.
 */
TEST(CorridorRay, AnswerIsIndependentOfCorridorWidth)
{
	const float far_wall = 89.5f;

	for (const float half_width : {12.f, 6.f, 3.f, 1.5f, 0.75f}) {
		flat_world();
		sdf_prim_t west = box(0.f, -(half_width + 0.5f), 4.f, 90.f, 0.5f, 4.f);
		sdf_prim_t east = box(0.f, (half_width + 0.5f), 4.f, 90.f, 0.5f, 4.f);
		sdf_prim_t cap  = box(90.f, 0.f, 4.f, 0.5f, half_width, 4.f);
		ASSERT_TRUE(sdf_scene_add(&west));
		ASSERT_TRUE(sdf_scene_add(&east));
		ASSERT_TRUE(sdf_scene_add(&cap));

		EXPECT_NEAR(sdf_sphere_trace(sdf_vec3{0.f, 0.f, 2.f}, kNorth, 200.f),
			    far_wall, 1e-2f) << "half width " << half_width;
	}
}

/* (f) yawed boxes, because the procedural wall lattice uses yaw */
TEST(CorridorRay, HandlesYawedBoxes)
{
	flat_world();

	/* Wall centred at north 20, rotated 45 degrees about the altitude axis.
	 * Half extents 0.5 by 12, so along the ray the half thickness projects
	 * to 0.5 / cos(45 deg). */
	const float c = std::cos(static_cast<float>(M_PI) / 4.f);
	const float s = std::sin(static_cast<float>(M_PI) / 4.f);
	sdf_prim_t w = box(20.f, 0.f, 4.f, 0.5f, 12.f, 4.f, c, s);
	ASSERT_TRUE(sdf_scene_add(&w));

	const float hit = sdf_sphere_trace(sdf_vec3{0.f, 0.f, 2.f}, kNorth, 100.f);
	EXPECT_NEAR(hit, 20.f - 0.5f / c, 1e-2f);
}
