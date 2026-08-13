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
 * @file terrain_map.hpp
 *
 * Loader for `.pxtm` real-elevation heightmaps from the SD card.
 *
 * `lib/terrain` can sample a grid of real DEM samples instead of (or on
 * top of) its procedural fBm surface, but it deliberately owns no heap
 * and does no file I/O — that keeps it pure C and keeps the bit-exact
 * shared-source contract with the companion viewer intact. This class is
 * the PX4-side half: it reads the file, owns the sample buffer, and hands
 * `lib/terrain` a borrowed pointer.
 *
 * ## Why a file and not a param or a generated header
 *
 * A 450 x 450 grid is 405 KB. That is far past what the parameter system
 * can carry and past what belongs in a 2 MB flash image, and the whole
 * point of real terrain is swapping sites without a reflash. So the map
 * is a file on the SD card, next to the logs.
 *
 * ## File format (little-endian, 64-byte header)
 *
 *   off  size  field
 *     0     4  magic         'PXTM'
 *     4     2  version       = 1
 *     6     2  flags         reserved, must be 0
 *     8     2  nx            samples along east   (2 .. 4096)
 *    10     2  ny            samples along north  (2 .. 4096)
 *    12     4  spacing_m     float, ground metres between samples
 *    16     8  origin_lat    double, WGS84 degrees of sample (0, 0)
 *    24     8  origin_lon    double, WGS84 degrees of sample (0, 0)
 *    32     4  origin_alt_m  float, elevation datum [m AMSL]
 *    36    28  reserved      zeros
 *    64  n*2  samples        int16 metres, row-major, row 0 = SOUTH,
 *                            column 0 = WEST
 *
 * The origin is stored as a `double` lat/lon deliberately. `SIH_LOC_LAT0`
 * and `SIH_LOC_LON0` are `ParamFloat`, and float32 resolves latitude to
 * only ~0.6 m and longitude to ~1.2 m at mid-latitudes. A surveyed site
 * needs better than that, so the fine registration lives in the file and
 * the params only anchor the local frame.
 *
 * ## Size cap
 *
 * `TERRAIN_MAP_MAX_BYTES` bounds the allocation so a corrupt header
 * cannot ask for a multi-megabyte malloc. It is set to 512 KB, which is
 * already above the largest contiguous block a live FMUv6 has free
 * (measured: ~441 KB), so the malloc is the real limit and this is just
 * the guard that keeps a bad file from trying.
 */

#pragma once

#include <lib/terrain/terrain.h>

#include <stdint.h>

/** Header size on disk. Samples begin at this offset. */
static constexpr unsigned TERRAIN_MAP_HEADER_BYTES = 64;

/** Refuse to allocate more than this for one map. See class comment. */
static constexpr unsigned TERRAIN_MAP_MAX_BYTES = 512u * 1024u;

/** Default location searched when the caller passes no explicit path. */
#define TERRAIN_MAP_DEFAULT_PATH "/fs/microsd/etc/terrain.pxtm"

/**
 * Owns one loaded heightmap and its lifetime against `lib/terrain`.
 *
 * Typical use from SIH:
 *
 *     if (_map.load(path, ref_lat, ref_lon)) { ... }   // installs grid
 *     _map.unload();                                   // clears it again
 *
 * `load()` installs the grid into `lib/terrain` on success. `unload()`
 * (and the destructor) clear it before freeing, so the library can never
 * hold a dangling pointer.
 */
class TerrainMap
{
public:
	TerrainMap() = default;
	~TerrainMap() { unload(); }

	TerrainMap(const TerrainMap &) = delete;
	TerrainMap &operator=(const TerrainMap &) = delete;

	/**
	 * Read a `.pxtm` file and install it as the active heightfield.
	 *
	 * The map's own origin (lat/lon from the file header) is projected
	 * against the vehicle's local frame reference so the terrain lands
	 * at its true geographic position rather than at local (0, 0). That
	 * projection is done by the caller and passed in as
	 * `origin_n` / `origin_e` — this class does no geo maths, so it
	 * stays testable on the host without PX4's geo library.
	 *
	 * Any previously loaded map is unloaded first, so calling `load()`
	 * repeatedly is safe and swaps maps atomically from the library's
	 * point of view.
	 *
	 * On any failure (missing file, bad magic, unsupported version,
	 * implausible dimensions, truncated samples, allocation failure)
	 * nothing is installed, the previous map stays unloaded, and the
	 * terrain library reverts to pure fBm.
	 *
	 * @param path      Filesystem path to the `.pxtm` file.
	 * @param origin_n  North coordinate of sample (0,0) [m from home].
	 * @param origin_e  East  coordinate of sample (0,0) [m from home].
	 * @return          true if a map is now installed.
	 */
	bool load(const char *path, float origin_n, float origin_e);

	/**
	 * Move the loaded map to a new position in the local frame.
	 *
	 * `load()` cannot place the map itself: the file's WGS84 origin is
	 * only known once the header has been read, and projecting it into
	 * the local frame needs PX4's geo library, which this class
	 * deliberately does not depend on. So the caller loads, reads
	 * `origin_lat()` / `origin_lon()`, projects, and calls this — no
	 * second pass over the file.
	 *
	 * No-op when nothing is loaded.
	 *
	 * @param origin_n  North coordinate of sample (0,0) [m from home].
	 * @param origin_e  East  coordinate of sample (0,0) [m from home].
	 */
	void set_origin(float origin_n, float origin_e);

	/** Clear the grid from `lib/terrain` and free the samples. */
	void unload();

	bool loaded() const { return _samples != nullptr; }

	/** Bytes held by the sample array (0 when not loaded). */
	unsigned size_bytes() const { return _bytes; }

	uint16_t nx() const { return _nx; }
	uint16_t ny() const { return _ny; }
	float spacing_m() const { return _spacing_m; }
	double origin_lat() const { return _origin_lat; }
	double origin_lon() const { return _origin_lon; }
	float origin_alt_m() const { return _origin_alt_m; }

private:
	int16_t *_samples{nullptr};
	unsigned _bytes{0};
	uint16_t _nx{0};
	uint16_t _ny{0};
	float    _spacing_m{0.f};
	double   _origin_lat{0.0};
	double   _origin_lon{0.0};
	float    _origin_alt_m{0.f};
};
