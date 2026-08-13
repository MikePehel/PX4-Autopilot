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
 * @file terrain_map.cpp
 *
 * See terrain_map.hpp for the file format and the rationale.
 */

#include "terrain_map.hpp"

#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

namespace
{

/* Little-endian readers. The format is fixed-LE so a map built on any
 * host loads on any target, and reading byte-wise avoids both alignment
 * faults on the packed header and any struct-padding surprises. */
uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

uint32_t rd_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	       | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

float rd_f32(const uint8_t *p)
{
	const uint32_t bits = rd_u32(p);
	float f;
	memcpy(&f, &bits, sizeof(f));
	return f;
}

double rd_f64(const uint8_t *p)
{
	const uint64_t bits = (uint64_t)rd_u32(p) | ((uint64_t)rd_u32(p + 4) << 32);
	double d;
	memcpy(&d, &bits, sizeof(d));
	return d;
}

/* Read exactly `n` bytes or fail. A single read() on a FAT SD card can
 * return short without being an error, so loop. */
bool read_exact(int fd, void *dst, size_t n)
{
	uint8_t *p = static_cast<uint8_t *>(dst);

	while (n > 0) {
		const ssize_t got = ::read(fd, p, n);

		if (got <= 0) {
			return false;
		}

		p += got;
		n -= (size_t)got;
	}

	return true;
}

} // namespace

void TerrainMap::set_origin(float origin_n, float origin_e)
{
	if (_samples == nullptr) {
		return;
	}

	const terrain_grid_t grid = {
		.samples = _samples,
		.nx = _nx,
		.ny = _ny,
		.spacing_m = _spacing_m,
		.origin_n = origin_n,
		.origin_e = origin_e,
	};

	terrain_set_grid(&grid);
}

void TerrainMap::unload()
{
	if (_samples == nullptr) {
		return;
	}

	/* Clear the library's borrowed pointer BEFORE freeing, so there is no
	 * window where terrain() could sample freed memory. */
	terrain_set_grid(nullptr);

	free(_samples);
	_samples = nullptr;
	_bytes = 0;
	_nx = 0;
	_ny = 0;
}

bool TerrainMap::load(const char *path, float origin_n, float origin_e)
{
	unload();

	if (path == nullptr) {
		return false;
	}

	const int fd = ::open(path, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("terrain map: cannot open %s", path);
		return false;
	}

	uint8_t hdr[TERRAIN_MAP_HEADER_BYTES];

	if (!read_exact(fd, hdr, sizeof(hdr))) {
		PX4_ERR("terrain map: short header");
		::close(fd);
		return false;
	}

	if (memcmp(hdr, "PXTM", 4) != 0) {
		PX4_ERR("terrain map: bad magic");
		::close(fd);
		return false;
	}

	const uint16_t version = rd_u16(hdr + 4);

	if (version != 1u) {
		PX4_ERR("terrain map: unsupported version %u", (unsigned)version);
		::close(fd);
		return false;
	}

	const uint16_t nx = rd_u16(hdr + 8);
	const uint16_t ny = rd_u16(hdr + 10);
	const float spacing = rd_f32(hdr + 12);

	/* lib/terrain requires >= 2 samples per axis to interpolate, and the
	 * 4096 cap keeps nx*ny*2 inside the byte budget checked below even
	 * before the multiply. */
	if (nx < 2u || ny < 2u || nx > 4096u || ny > 4096u
	    || !(spacing > 0.f) || !(spacing < 100000.f)) {
		PX4_ERR("terrain map: implausible geometry %ux%u @ %.2f m",
			(unsigned)nx, (unsigned)ny, (double)spacing);
		::close(fd);
		return false;
	}

	const unsigned bytes = (unsigned)nx * (unsigned)ny * sizeof(int16_t);

	if (bytes > TERRAIN_MAP_MAX_BYTES) {
		PX4_ERR("terrain map: %u KB exceeds %u KB cap",
			bytes / 1024u, TERRAIN_MAP_MAX_BYTES / 1024u);
		::close(fd);
		return false;
	}

	int16_t *samples = static_cast<int16_t *>(malloc(bytes));

	if (samples == nullptr) {
		PX4_ERR("terrain map: out of memory for %u KB", bytes / 1024u);
		::close(fd);
		return false;
	}

	if (!read_exact(fd, samples, bytes)) {
		PX4_ERR("terrain map: truncated, wanted %u B of samples", bytes);
		free(samples);
		::close(fd);
		return false;
	}

	::close(fd);

	_samples = samples;
	_bytes = bytes;
	_nx = nx;
	_ny = ny;
	_spacing_m = spacing;
	_origin_lat = rd_f64(hdr + 16);
	_origin_lon = rd_f64(hdr + 24);
	_origin_alt_m = rd_f32(hdr + 32);

	const terrain_grid_t grid = {
		.samples = _samples,
		.nx = _nx,
		.ny = _ny,
		.spacing_m = _spacing_m,
		.origin_n = origin_n,
		.origin_e = origin_e,
	};

	terrain_set_grid(&grid);

	PX4_INFO("terrain map: %ux%u @ %.1f m = %.1f x %.1f km, %u KB, origin %.6f %.6f",
		 (unsigned)_nx, (unsigned)_ny, (double)_spacing_m,
		 (double)(_nx * _spacing_m / 1000.f),
		 (double)(_ny * _spacing_m / 1000.f),
		 _bytes / 1024u, _origin_lat, _origin_lon);

	return true;
}
