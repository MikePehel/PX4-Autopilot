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
 * @file corridor_demo.hpp
 *
 * The hand-built walls that corridor mode (SIH_TERR_EN = 4) flies among.
 *
 * Walls are solid boxes, the same `SDF_PRIM_BOX` the procedural lattice
 * already uses. They are objects in the world, not the boundary of a volume
 * the vehicle inhabits, so they compose with terrain by min()-union and the
 * drone can fly out of an end and land on open ground. Arrange enough of
 * them and the clear space between reads as rooms and corridors.
 *
 * Layout: a 90 m by 24 m corridor, walls 8 m tall and 1 m thick, split into
 * three bays by cross walls at north -15 and +15 that each carry a centred
 * 8 m doorway.
 *
 *   north -45                    0                    +45
 *         +==========================================+   east +12.5
 *         |        |             |                   |
 *         |        |    home     |                   |
 *         +==========================================+   east -12.5
 *
 * From the spawn point at local (0, 0) that gives 12 m to either side wall
 * and 15 m to either cross wall, which is what the self-check verifies and
 * what the SITL and hardware runs compare against.
 *
 * The 24 m clear width is deliberate. Collision prevention refuses to
 * translate when every bin reads inside its braking threshold, so a corridor
 * has to be wider than twice that threshold before the vehicle sits clear on
 * both sides. 24 m clears the default 10 m setting with room to spare.
 */

#pragma once

#if defined(HAWKEYE_VENDORED)
#  include "terrain_sdf.h"
#else
#  include <lib/terrain_sdf/terrain_sdf.h>
#endif

/** @return number of walls in the demo building. */
int corridor_demo_wall_count(void);

/**
 * Fetch one wall as a scene primitive.
 *
 * Exposed so the viewer can draw exactly the boxes the simulation traces
 * against, rather than keeping a second description that can drift.
 *
 * @param index  Wall index in [0, corridor_demo_wall_count()).
 * @param out    Out, the primitive.
 * @return       false on a bad index or NULL out.
 */
bool corridor_demo_wall(int index, sdf_prim_t *out);

/**
 * Install every wall into the SDF scene, then fire the probes the sensors
 * will fire and check the known distances.
 *
 * @return true when the building installed and answered correctly. On false
 *         the scene is left cleared rather than flying geometry that lies.
 */
bool corridor_demo_install_and_check(void);
