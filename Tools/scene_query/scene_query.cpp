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
 * @file scene_query.c
 *
 * Host-side dump of the procedural SDF wall lattice for a given seed.
 *
 * Links against the same lib/terrain + lib/terrain_sdf source files PX4
 * compiles, so the (seed, cell_i, cell_j) → primitive mapping printed
 * here is bit-identical to what SIH materialises at runtime. The
 * companion viewer consumes the same source, so the bytes match there
 * too.
 *
 * Walls are not indexed by a count; they fall out of a cell grid
 * iterated by SIH. This tool walks the same grid and prints each
 * cell's `scene_wall_at_cell` output (or "(empty)" for cells that
 * either fail the presence check or fall inside the home-clearance
 * radius).
 *
 * Usage:
 *   scene_query --seed N [--cell-radius R]
 *
 * Output (one block per occupied cell):
 *   cell[i, j]: type=box
 *     cell_center=(N, E)
 *     center=(N, E, alt)
 *     extent=(half_n_local, half_e_local, half_alt)
 *     length=L m  thickness=T m  height=H m  yaw=Y deg
 *     ctr_dist_from_home = D m  bearing_from_E = B deg
 *
 * Test fixtures pin SIH_TERR_SEED to a known value, run this tool, and
 * paste the constants of an occupied cell into the test source as
 * expected coordinates (see test/sih_terrain/sdf_wall_avoidance_test.py
 * for the pinned-seed fixture).
 */

#include "../../src/lib/terrain/terrain.h"
#include "../../src/lib/terrain_sdf/scene.h"
#include "../../src/lib/terrain_sdf/terrain_sdf.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *prim_type_name(sdf_prim_type_t t)
{
	switch (t) {
	case SDF_PRIM_BOX:      return "box";

	case SDF_PRIM_CYLINDER: return "cylinder";

	case SDF_PRIM_SPHERE:   return "sphere";

	case SDF_PRIM_PLANE:    return "plane";

	default:                return "?";
	}
}

static void print_wall(int32_t i_cell, int32_t j_cell, const sdf_prim_t *w)
{
	const float cell_n   = (float)i_cell * WALL_CELL_SPACING_M;
	const float cell_e   = (float)j_cell * WALL_CELL_SPACING_M;
	const float length    = 2.f * w->extent.n;
	const float thickness = 2.f * w->extent.e;
	const float height    = 2.f * w->extent.alt;
	const float yaw_deg   = atan2f(w->sin_yaw, w->cos_yaw) * 180.f / 3.14159265358979f;
	const float ctr_dist  = sqrtf((w->center.n * w->center.n)
				      + (w->center.e * w->center.e));
	const float bearing_E_deg = atan2f(w->center.n, w->center.e) * 180.f / 3.14159265358979f;

	printf("cell[%+d, %+d]: type=%s\n", i_cell, j_cell, prim_type_name(w->type));
	printf("  cell_center = (%+9.3f, %+9.3f)\n", (double)cell_n, (double)cell_e);
	printf("  center      = (%+9.3f, %+9.3f, %+9.3f)\n",
	       (double)w->center.n, (double)w->center.e, (double)w->center.alt);
	printf("  extent_local= (%+9.3f, %+9.3f, %+9.3f)\n",
	       (double)w->extent.n, (double)w->extent.e, (double)w->extent.alt);
	printf("  length=%6.3f m  thickness=%6.3f m  height=%6.3f m  yaw=%+7.2f deg\n",
	       (double)length, (double)thickness, (double)height, (double)yaw_deg);
	printf("  ctr_dist_from_home = %6.3f m  bearing_from_E = %+7.2f deg\n",
	       (double)ctr_dist, (double)bearing_E_deg);
}

static void print_usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s --seed N [--cell-radius R]\n"
		"  --seed N         SIH_TERR_SEED value (int32)\n"
		"  --cell-radius R  cell-grid radius (default WALL_CELL_RADIUS_CELLS = %d)\n",
		prog, WALL_CELL_RADIUS_CELLS);
}

int main(int argc, char **argv)
{
	int seed = -1;
	int radius = WALL_CELL_RADIUS_CELLS;

	for (int k = 1; k < argc; k++) {
		if (strcmp(argv[k], "--seed") == 0 && k + 1 < argc) {
			seed = atoi(argv[++k]);

		} else if (strcmp(argv[k], "--cell-radius") == 0 && k + 1 < argc) {
			radius = atoi(argv[++k]);

		} else if (strcmp(argv[k], "--help") == 0 || strcmp(argv[k], "-h") == 0) {
			print_usage(argv[0]);
			return 0;

		} else {
			fprintf(stderr, "unknown argument: %s\n\n", argv[k]);
			print_usage(argv[0]);
			return 2;
		}
	}

	if (seed < 0) {
		fprintf(stderr, "error: --seed is required\n\n");
		print_usage(argv[0]);
		return 2;
	}

	if (radius < 0 || radius > 50) {
		fprintf(stderr, "error: --cell-radius must be in [0, 50]\n");
		return 2;
	}

	/* The scene generator only reads terrain_get_seed(); other
	 * SIH_TERR_* knobs are irrelevant for wall geometry. Use the same
	 * call pattern SIH uses in parameters_updated() so any future
	 * seed-mixing behaviour stays consistent. AMP and wavelength here
	 * are arbitrary — terrain elevation is not sampled by this tool. */
	terrain_set_params(50.f, 200.f, seed, 0.f, 0.f, 0.f);

	printf("# scene_query --seed %d --cell-radius %d\n", seed, radius);
	printf("# WALL_CELL_SPACING_M = %.1f, WALL_PRESENCE_P = %.2f\n",
	       (double)WALL_CELL_SPACING_M, (double)WALL_PRESENCE_P);
	printf("# WALL_HOME_CLEARANCE_M = %.1f, WALL_THICKNESS_M = %.1f\n",
	       (double)WALL_HOME_CLEARANCE_M, (double)WALL_THICKNESS_M);
	printf("# WALL length in [%.1f, %.1f) m, height in [%.1f, %.1f] m, yaw in [0, 180) deg\n\n",
	       (double)WALL_MIN_LEN_M, (double)WALL_MAX_LEN_M,
	       (double)WALL_HEIGHT_MIN_M, (double)WALL_HEIGHT_MAX_M);

	int n_occupied = 0;
	int n_total    = 0;

	for (int32_t i = -radius; i <= radius; i++) {
		for (int32_t j = -radius; j <= radius; j++) {
			sdf_prim_t w;
			n_total++;

			if (scene_wall_at_cell(i, j, &w)) {
				print_wall(i, j, &w);
				n_occupied++;
				printf("\n");
			}
		}
	}

	printf("# %d / %d cells occupied (%.1f %%)\n",
	       n_occupied, n_total,
	       100.0 * (double)n_occupied / (double)n_total);

	return 0;
}
