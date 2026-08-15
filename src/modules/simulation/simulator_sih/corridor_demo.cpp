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

#include "corridor_demo.hpp"

/*
 * Shared source with the Hawkeye viewer so the drawn walls and the flown
 * walls are one definition. HAWKEYE_VENDORED swaps PX4's logging macros.
 */
#if defined(HAWKEYE_VENDORED)
#  include <stdio.h>
#  define PX4_ERR(...)  do { fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)
#  define PX4_INFO(...) do { printf(__VA_ARGS__); putchar('\n'); } while (0)
#else
#  include <px4_platform_common/log.h>
#endif

#include <math.h>

namespace
{

/*
 * Wall thickness, matching the procedural lattice in scene.h so indoor and
 * outdoor walls read the same to a rangefinder.
 */
constexpr float kThick  = 1.0f;
constexpr float kHeight = 8.0f;      /* wall top, floor is the ground */
constexpr float kHalfW  = 12.0f;     /* corridor half width, 24 m clear */
constexpr float kHalfL  = 45.0f;     /* corridor half length, 90 m long  */
constexpr float kDoor   = 4.0f;      /* half width of each doorway gap   */

/*
 * A wall is a solid box, not a boundary. `corridor_demo_walls` is the whole
 * building: arrange enough of them and the clear space between reads as
 * rooms and corridors. Nothing here describes the empty volume, which is
 * why the vehicle can fly out of an end and land on open ground.
 *
 *   north -45                    0                    +45
 *         +==========================================+      east +12.5
 *         |        |                    |            |
 *         |  gap   |       gap          |            |   <- cross walls with
 *         |        |                    |            |      doorway gaps
 *         +==========================================+      east -12.5
 *
 * Cross walls at north -15 and +15 each have a centred doorway, so the
 * three bays are connected but a ray fired at a wall stops at it.
 */
struct WallSpec {
	float centre_n;
	float centre_e;
	float centre_alt;
	float half_n;
	float half_e;
	float half_alt;
};

constexpr float kSideOffset = kHalfW + kThick * 0.5f;

constexpr float kWallMidAlt = kHeight * 0.5f;
constexpr float kJamb       = (kHalfW - kDoor) * 0.5f;   /* half width of a door jamb */
constexpr float kJambOffset = kDoor + kJamb;             /* its centre, east of centreline */

constexpr WallSpec kWalls[] = {
	/* Two long side walls, floor to ceiling. */
	{   0.f, -kSideOffset, kWallMidAlt, kHalfL,        kThick * 0.5f, kWallMidAlt },
	{   0.f,  kSideOffset, kWallMidAlt, kHalfL,        kThick * 0.5f, kWallMidAlt },

	/* Two end caps. */
	{ -kHalfL - kThick * 0.5f, 0.f, kWallMidAlt, kThick * 0.5f, kHalfW, kWallMidAlt },
	{  kHalfL + kThick * 0.5f, 0.f, kWallMidAlt, kThick * 0.5f, kHalfW, kWallMidAlt },

	/* Cross wall at north -15, split around a centred doorway. */
	{ -15.f, -kJambOffset, kWallMidAlt, kThick * 0.5f, kJamb, kWallMidAlt },
	{ -15.f,  kJambOffset, kWallMidAlt, kThick * 0.5f, kJamb, kWallMidAlt },

	/* Cross wall at north +15, same split. */
	{  15.f, -kJambOffset, kWallMidAlt, kThick * 0.5f, kJamb, kWallMidAlt },
	{  15.f,  kJambOffset, kWallMidAlt, kThick * 0.5f, kJamb, kWallMidAlt },

	/* Ceiling slab. The floor is the terrain ground plane, which is already
	 * real geometry every ray hits, so a building needs a lid and not a
	 * base. Sits on top of the walls rather than inside them, so the clear
	 * height stays exactly kHeight. */
	{ 0.f, 0.f, kHeight + kThick * 0.5f, kHalfL, kSideOffset, kThick * 0.5f },
};

constexpr int kWallCount = (int)(sizeof(kWalls) / sizeof(kWalls[0]));

/*
 * Distances a beam must report, by construction. Each probe has its own
 * origin because the interesting cases are not all on the centreline: a ray
 * fired north from the centre goes straight through both doorways, so it is
 * the off-centre one that proves the cross walls are solid.
 */
struct Probe {
	const char *what;
	sdf_vec3    from;
	sdf_vec3    dir;
	float       expect_m;
};

constexpr float kProbeAlt = 2.f;

constexpr Probe kProbes[] = {
	/* Side walls, from the centreline. */
	{"east",       { 0.f, 0.f, kProbeAlt}, { 0.f,  1.f,  0.f}, kHalfW},
	{"west",       { 0.f, 0.f, kProbeAlt}, { 0.f, -1.f,  0.f}, kHalfW},

	/* Straight up the centreline the doorways are open, so this reaches
	 * the far end cap rather than the nearest cross wall. */
	{"north door", { 0.f, 0.f, kProbeAlt}, { 1.f,  0.f,  0.f}, kHalfL},
	{"south door", { 0.f, 0.f, kProbeAlt}, {-1.f,  0.f,  0.f}, kHalfL},

	/* Offset east of the doorway, the same ray hits a jamb. A ray stops at
	 * the near FACE, so the distance is the wall centreline less half its
	 * thickness, the same reason the side walls read kHalfW and not
	 * kSideOffset. */
	{"north jamb", { 0.f, 8.f, kProbeAlt}, { 1.f,  0.f,  0.f}, 15.f - kThick * 0.5f},
	{"south jamb", { 0.f, 8.f, kProbeAlt}, {-1.f,  0.f,  0.f}, 15.f - kThick * 0.5f},

	/* Ceiling above, ground below. */
	{"up",         { 0.f, 0.f, kProbeAlt}, { 0.f,  0.f,  1.f}, kHeight - kProbeAlt},
	{"down",       { 0.f, 0.f, kProbeAlt}, { 0.f,  0.f, -1.f}, kProbeAlt},
};

} // namespace

int corridor_demo_wall_count(void)
{
	return kWallCount;
}

bool corridor_demo_wall(int index, sdf_prim_t *out)
{
	if (index < 0 || index >= kWallCount || out == nullptr) {
		return false;
	}

	const WallSpec &w = kWalls[index];

	out->type    = SDF_PRIM_BOX;
	out->center  = sdf_vec3{ w.centre_n, w.centre_e, w.centre_alt };
	out->extent  = sdf_vec3{ w.half_n, w.half_e, w.half_alt };
	out->cos_yaw = 1.f;   /* axis aligned, so the box test skips rotation */
	out->sin_yaw = 0.f;
	return true;
}

bool corridor_demo_install_and_check(void)
{
	sdf_scene_clear();

	for (int i = 0; i < kWallCount; ++i) {
		sdf_prim_t w{};

		if (!corridor_demo_wall(i, &w) || !sdf_scene_add(&w)) {
			PX4_ERR("corridor: wall %d did not fit the scene list", i);
			sdf_scene_clear();
			return false;
		}
	}

	/*
	 * Fire the probes the sensors will fire. A building that answers wrong
	 * here would make every rangefinder and ring bin report a plausible
	 * wrong distance with no other symptom, so it is worth the microseconds
	 * at mode-switch time.
	 */
	for (const Probe &p : kProbes) {
		const float got = sdf_sphere_trace(p.from, p.dir, 200.f);

		if (fabsf(got - p.expect_m) > 0.05f) {
			PX4_ERR("corridor: %s probe read %.2f m, expected %.2f m",
				p.what, static_cast<double>(got),
				static_cast<double>(p.expect_m));
			sdf_scene_clear();
			return false;
		}
	}

	PX4_INFO("corridor: %d walls installed, self-check passed", kWallCount);
	return true;
}
