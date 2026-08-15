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
 * @file golden_vectors.cpp
 *
 * Bit-exact regression harness for lib/terrain and lib/terrain_sdf.
 *
 * Both libraries are pure functions of their parameters, compiled into the
 * firmware, into the Hawkeye viewer and into WASM. That only holds if every
 * target produces identical bits, and a single compiler flag difference is
 * enough to break it. Decimal comparison hides exactly that class of fault,
 * so every value is emitted as its raw IEEE-754 bit pattern in hex.
 *
 * Two modes:
 *
 *   golden_vectors            verify against golden_reference.h, exit 1 on
 *                             any mismatch
 *   golden_vectors --generate print a fresh golden_reference.h to stdout
 *
 * Regenerate only when a change to the numerical output is intended, and say
 * so in the commit message. A silent regeneration defeats the whole point.
 *
 * Coverage is deliberately wider than the corridor work that prompted it,
 * because the risk being managed is that a change aimed at one mode moves
 * the numbers in another.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <initializer_list>

#include "../../terrain.h"
#include "../../../terrain_sdf/terrain_sdf.h"
#include "../../../terrain_sdf/scene.h"

namespace
{

/* Raw bits, because a one-bit drift is the fault being hunted. */
uint32_t bits(float f)
{
	uint32_t u;
	std::memcpy(&u, &f, sizeof(u));
	return u;
}

struct Sink {
	bool generate;
	int  index;
	int  mismatches;
	int  checked;
};

Sink g_sink;

} // namespace

/* The reference table. Regenerate with --generate, never by hand. */
#include "golden_reference.h"

namespace
{

void emit(float v)
{
	const uint32_t b = bits(v);

	if (g_sink.generate) {
		if (g_sink.index % 6 == 0) { std::printf("\n\t"); }

		std::printf("0x%08xu, ", b);

	} else {
		if (g_sink.index < kGoldenCount) {
			if (kGolden[g_sink.index] != b) {
				if (g_sink.mismatches < 12) {
					std::printf("  mismatch at %d: golden 0x%08x, got 0x%08x (%.9g)\n",
						    g_sink.index, kGolden[g_sink.index], b, (double) v);
				}

				g_sink.mismatches++;
			}

			g_sink.checked++;
		}
	}

	g_sink.index++;
}

/*
 * Sweep every pure output of both libraries.
 *
 * The point of covering rays that miss, rays that graze and rays fired from
 * inside geometry is that those are the paths where a tracer change is most
 * likely to move a value without moving the obvious cases.
 */
void sweep_terrain()
{
	const int   seeds[]  = {0, 3, 42, 622};
	const float amps[]   = {0.f, 12.f, 60.f};
	const float planes[] = {0.f, 7.5f, -12.f};

	for (int seed : seeds) {
		for (float amp : amps) {
			for (float plane : planes) {
				terrain_set_params(amp, 200.f, seed, 0.f, 0.f, plane);

				for (int i = -4; i <= 4; ++i) {
					for (int j = -4; j <= 4; ++j) {
						const float n = i * 37.5f;
						const float e = j * 37.5f;
						emit(terrain(n, e));

						float dn = 0.f, de = 0.f;
						terrain_gradient(n, e, &dn, &de);
						emit(dn);
						emit(de);
					}
				}

				/* Analytic raycast, including tilted and upward beams. */
				const float dirs[][3] = {
					{0.f, 0.f, -1.f}, {0.2f, 0.f, -0.98f},
					{0.f, 0.5f, -0.87f}, {0.7f, 0.7f, -0.14f},
					{0.f, 0.f, 1.f},
				};

				for (const auto &d : dirs) {
					emit(raycast(0.f, 0.f, 120.f, d[0], d[1], d[2], 400.f));
					emit(raycast(150.f, -90.f, 8.f, d[0], d[1], d[2], 400.f));
				}
			}
		}
	}
}

void sweep_walls()
{
	const int seeds[] = {0, 3, 42, 622};

	for (int seed : seeds) {
		terrain_set_params(0.f, 200.f, seed, 0.f, 0.f, 0.f);

		for (int32_t i = -3; i <= 3; ++i) {
			for (int32_t j = -3; j <= 3; ++j) {
				sdf_prim_t w{};
				const bool has = scene_wall_at_cell(i, j, &w);
				emit(has ? 1.f : 0.f);

				if (has) {
					emit(w.center.n);
					emit(w.center.e);
					emit(w.center.alt);
					emit(w.extent.n);
					emit(w.extent.e);
					emit(w.extent.alt);
					emit(w.cos_yaw);
					emit(w.sin_yaw);
				}
			}
		}
	}
}

void sweep_tracer()
{
	const int seeds[] = {0, 42};

	for (int seed : seeds) {
		for (float amp : {0.f, 30.f}) {
			for (bool walls : {false, true}) {
				terrain_set_params(amp, 200.f, seed, 0.f, 0.f, 0.f);
				sdf_scene_clear();
				sdf_walls_set_enabled(walls);

				/* A materialised box, so the exact path is covered too. */
				sdf_prim_t b{};
				b.type    = SDF_PRIM_BOX;
				b.center  = sdf_vec3{40.f, 0.f, 6.f};
				b.extent  = sdf_vec3{1.f, 15.f, 6.f};
				b.cos_yaw = 1.f;
				b.sin_yaw = 0.f;
				sdf_scene_add(&b);

				/* A yawed one, to cover the rotation path. */
				sdf_prim_t y{};
				y.type    = SDF_PRIM_BOX;
				y.center  = sdf_vec3{-25.f, 18.f, 5.f};
				y.extent  = sdf_vec3{0.75f, 9.f, 5.f};
				y.cos_yaw = 0.70710678f;
				y.sin_yaw = 0.70710678f;
				sdf_scene_add(&y);

				const float alts[] = {0.05f, 0.4f, 1.f, 3.f, 12.f, 60.f};
				const float dirs[][3] = {
					{1.f, 0.f, 0.f}, {-1.f, 0.f, 0.f},
					{0.f, 1.f, 0.f}, {0.f, -1.f, 0.f},
					{0.f, 0.f, 1.f}, {0.f, 0.f, -1.f},
					{0.70710678f, 0.70710678f, 0.f},
					{0.5773f, -0.5773f, -0.5773f},
				};

				for (float alt : alts) {
					for (const auto &d : dirs) {
						emit(sdf_sphere_trace(sdf_vec3{0.f, 0.f, alt},
								      sdf_vec3{d[0], d[1], d[2]}, 150.f));
						emit(sdf_sphere_trace(sdf_vec3{ -60.f, 22.f, alt},
								      sdf_vec3{d[0], d[1], d[2]}, 150.f));
					}
				}
			}
		}
	}
}

/* A small synthetic grid, so the DEM path is covered without a file. */
int16_t g_grid[32 * 32];

void sweep_grid()
{
	for (int r = 0; r < 32; ++r) {
		for (int c = 0; c < 32; ++c) {
			g_grid[r * 32 + c] = (int16_t)(400.f
						       + 30.f * std::sin(r * 0.31f)
						       + 18.f * std::cos(c * 0.21f));
		}
	}

	terrain_grid_t grid{};
	grid.samples   = g_grid;
	grid.nx        = 32;
	grid.ny        = 32;
	grid.spacing_m = 90.f;
	grid.origin_n  = -1440.f;
	grid.origin_e  = -1440.f;

	terrain_set_params(0.f, 200.f, 0, 0.f, 0.f, 0.f);
	terrain_set_grid(&grid);

	for (int i = -6; i <= 6; ++i) {
		for (int j = -6; j <= 6; ++j) {
			const float n = i * 240.f;
			const float e = j * 240.f;
			emit(terrain(n, e));

			float dn = 0.f, de = 0.f;
			terrain_gradient(n, e, &dn, &de);
			emit(dn);
			emit(de);
		}
	}

	sdf_scene_clear();
	sdf_walls_set_enabled(false);

	for (float alt : {2.f, 25.f, 200.f}) {
		emit(sdf_sphere_trace(sdf_vec3{0.f, 0.f, alt}, sdf_vec3{0.f, 0.f, -1.f}, 500.f));
		emit(sdf_sphere_trace(sdf_vec3{0.f, 0.f, alt}, sdf_vec3{1.f, 0.f, 0.f}, 500.f));
	}

	terrain_set_grid(nullptr);
}

} // namespace

int main(int argc, char **argv)
{
	g_sink.generate = (argc > 1 && std::strcmp(argv[1], "--generate") == 0);

	if (g_sink.generate) {
		std::printf("/* Generated by golden_vectors --generate. Do not hand edit. */\n");
		std::printf("namespace {\nconst uint32_t kGolden[] = {");
	}

	sweep_terrain();
	sweep_walls();
	sweep_tracer();
	sweep_grid();

	if (g_sink.generate) {
		std::printf("\n};\nconst int kGoldenCount = %d;\n} // namespace\n", g_sink.index);
		return 0;
	}

	if (g_sink.index != kGoldenCount) {
		std::printf("FAIL: produced %d values, reference has %d\n",
			    g_sink.index, kGoldenCount);
		return 1;
	}

	if (g_sink.mismatches != 0) {
		std::printf("FAIL: %d of %d values differ\n", g_sink.mismatches, g_sink.checked);
		return 1;
	}

	std::printf("golden vectors: %d values bit-identical\n", g_sink.checked);
	return 0;
}
