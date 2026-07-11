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
 * To run this test only use: make tests TESTFILTER=terrain
 *
 * Test plan:
 *   (a) terrain(0, 0) == 0 within float epsilon — home offset works
 *   (b) terrain is deterministic — same (N, E, seed) -> same value
 *   (c) raycast returns altitude on a straight-down beam over flat ground
 *   (d) raycast returns max_t when the beam is outside the analytical
 *       envelope (≤5° off straight down). Arbitrary-direction beams
 *       are the SDF tracer's job, not raycast()'s.
 *   (e) raycast returns max_t when the beam misses (upward / range too
 *       short)
 *   (f) per-call cost: 1000 terrain() calls under 10 ms on host
 *
 * Each test calls terrain_set_params() up front since the library has
 * static configuration state shared across tests; gtest does not
 * guarantee test ordering, so don't rely on a previous test's params.
 */

#include <gtest/gtest.h>
#include <lib/terrain/terrain.h>

#include <chrono>
#include <cmath>

namespace
{

constexpr float kWavelength = 200.f;

void configure_flat()
{
	terrain_set_params(0.f, kWavelength, 0, 0.f, 0.f, 0.f);
}

void configure_hilly(int seed, float amp = 25.f)
{
	terrain_set_params(amp, kWavelength, seed, 0.f, 0.f, 0.f);
}

} // namespace

/* --------------------------------------------------------------------------
 * (a) terrain(0, 0) == 0 — home offset works
 * -------------------------------------------------------------------------- */

TEST(TestTerrain, HomeOriginIsExactlyZero)
{
	for (int seed : {0, 1, 7, 42, 1000}) {
		configure_hilly(seed);
		EXPECT_NEAR(terrain(0.f, 0.f), 0.f, 1e-3f)
			<< "seed=" << seed;
	}
}

TEST(TestTerrain, HomeOriginIsZeroWhenAmpZero)
{
	configure_flat();
	EXPECT_FLOAT_EQ(terrain(0.f, 0.f),   0.f);
	EXPECT_FLOAT_EQ(terrain(100.f, 50.f), 0.f);
	EXPECT_FLOAT_EQ(terrain(-1234.f, 5678.f), 0.f);
}

/* --------------------------------------------------------------------------
 * (b) terrain is deterministic — same inputs -> same value
 * -------------------------------------------------------------------------- */

TEST(TestTerrain, IsBitExactRepeatableForSameSeed)
{
	configure_hilly(42);

	const float a1 = terrain(17.5f, -89.25f);
	const float a2 = terrain(17.5f, -89.25f);
	EXPECT_EQ(a1, a2);

	/* Reconfigure with a different seed, then back. The recomputed
	 * home offset must reproduce the same hashed lattice. */
	configure_hilly(7);
	configure_hilly(42);

	const float a3 = terrain(17.5f, -89.25f);
	EXPECT_EQ(a1, a3);
}

TEST(TestTerrain, DifferentSeedsProduceDifferentValues)
{
	configure_hilly(0);
	const float h0 = terrain(123.f, -456.f);

	configure_hilly(1);
	const float h1 = terrain(123.f, -456.f);

	EXPECT_NE(h0, h1);
}

TEST(TestTerrain, GradientMatchesFiniteDifference)
{
	configure_hilly(3);

	/* Pick a non-origin point so we are not at the home zero. */
	const float n = 80.f;
	const float e = -110.f;
	const float eps = 0.5f;

	float dn_analytic = 0.f;
	float de_analytic = 0.f;
	terrain_gradient(n, e, &dn_analytic, &de_analytic);

	const float dn_fd = (terrain(n + eps, e) - terrain(n - eps, e)) / (2.f * eps);
	const float de_fd = (terrain(n, e + eps) - terrain(n, e - eps)) / (2.f * eps);

	/* Tolerance is loose-ish because the erosion divisor is treated as
	 * a constant in the analytic gradient (standard Quilez approx) so
	 * the analytic and FD gradients can differ by a few percent. */
	EXPECT_NEAR(dn_analytic, dn_fd, 0.1f) << "n-component";
	EXPECT_NEAR(de_analytic, de_fd, 0.1f) << "e-component";
}

/* --------------------------------------------------------------------------
 * (c) raycast returns altitude on a straight-down beam over flat ground
 *
 * Altitude-up frame convention: dir_alt = -1.f for straight-down.
 * -------------------------------------------------------------------------- */

TEST(TestTerrain, RaycastFlatStraightDownReturnsAltitude)
{
	configure_flat();
	const float t = raycast(0.f, 0.f, 42.5f, 0.f, 0.f, -1.f, 1000.f);
	EXPECT_FLOAT_EQ(t, 42.5f);
}

/* --------------------------------------------------------------------------
 * (d) raycast returns max_t for beams outside the analytical envelope
 *
 * raycast() handles only near-vertical downward beams (≤ ~5° off
 * straight-down). Beyond that envelope it returns max_t so callers
 * re-issue via sdf_sphere_trace. Verified for: any tilt > 5°, any
 * upward beam.
 * -------------------------------------------------------------------------- */

TEST(TestTerrain, RaycastTiltedBeamOutsideEnvelopeReturnsMaxT)
{
	configure_flat();
	const float max_t = 1000.f;
	/* 30° off vertical: dir = (0, sin(30°), -cos(30°)) = (0, 0.5, -0.866).
	 * Far outside the ~5° analytical envelope — must return max_t. */
	const float t = raycast(0.f, 0.f, 50.f, 0.f, 0.5f, -0.866025f, max_t);
	EXPECT_FLOAT_EQ(t, max_t);
}

TEST(TestTerrain, RaycastNearlyVerticalInsideEnvelopeReturnsHit)
{
	configure_flat();
	/* 3° off vertical: dir = (0, sin(3°), -cos(3°)) ≈ (0, 0.0523, -0.9986).
	 * Just inside the 5° envelope (cos(5°) = 0.9962). Hit at
	 * t = origin_alt / cos(3°) ≈ 30.04 m. */
	const float dir_e   =  0.05234f;
	const float dir_alt = -0.99863f;
	const float t = raycast(0.f, 0.f, 30.f, 0.f, dir_e, dir_alt, 1000.f);
	EXPECT_NEAR(t, 30.f / 0.99863f, 0.01f);
}

/* --------------------------------------------------------------------------
 * (e) raycast returns max_t when the beam misses
 * -------------------------------------------------------------------------- */

TEST(TestTerrain, RaycastUpwardBeamReturnsMaxT)
{
	configure_flat();
	const float max_t = 250.f;
	/* Upward beam (dir_alt = +1) from above flat ground — out of
	 * envelope by sign alone (cos_tilt = -1 < 0). */
	const float t = raycast(0.f, 0.f, 10.f, 0.f, 0.f, 1.f, max_t);
	EXPECT_FLOAT_EQ(t, max_t);
}

TEST(TestTerrain, RaycastTooShortRangeReturnsMaxT)
{
	configure_flat();
	const float max_t = 5.f;
	/* Straight down from 100 m with max_t = 5 m: should not reach
	 * the ground within range. */
	const float t = raycast(0.f, 0.f, 100.f, 0.f, 0.f, -1.f, max_t);
	EXPECT_FLOAT_EQ(t, max_t);
}

/* --------------------------------------------------------------------------
 * (f) per-call cost: 1000 terrain() calls under 10 ms on host
 * -------------------------------------------------------------------------- */

TEST(TestTerrain, ThousandCallsUnderTenMs)
{
	configure_hilly(123);

	volatile float sink = 0.f;
	const auto t0 = std::chrono::steady_clock::now();

	for (int i = 0; i < 1000; ++i) {
		/* Spread inputs across the noise field so the compiler can't
		 * hoist the call. */
		const float n = static_cast<float>(i) * 0.317f;
		const float e = static_cast<float>(1000 - i) * 0.213f;
		sink += terrain(n, e);
	}

	const auto t1 = std::chrono::steady_clock::now();
	const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

	(void)sink;
	EXPECT_LT(us, 10000) << "1000 terrain() calls took " << us << " us (budget 10 000 us)";
}

TEST(TestTerrain, LookupTableSizeIsZero)
{
	EXPECT_EQ(terrain_seed_lookup_table_size(), 0u);
}
