#!/usr/bin/env python3
"""Convert an SRTM .hgt tile into a .pxtm heightmap for SIH.

The output drops on the flight controller's SD card and is read by
TerrainMap (src/modules/simulation/simulator_sih/terrain_map.cpp), which
hands it to lib/terrain as the heightfield the SIH sensors raycast
against. No reflash needed to change sites.

Sources for .hgt tiles (free, no auth):
    https://s3.amazonaws.com/elevation-tiles-prod/skadi/N46/N46E008.hgt.gz

Example:
    ./dem_to_pxtm.py N46E008.hgt --centre 46.55 8.02 --span-km 12 \\
        --spacing 240 -o terrain.pxtm

Only dependency is numpy.
"""
import argparse
import math
import struct
import sys

import numpy as np

MAGIC = b"PXTM"
VERSION = 1
HEADER_BYTES = 64
MAX_BYTES = 512 * 1024          # must match TERRAIN_MAP_MAX_BYTES
SRTM_VOID = -32768


def load_hgt(path):
    """Read a raw SRTM tile. Side length is implied by the file size."""
    raw = np.fromfile(path, dtype=">i2")
    side = int(round(math.sqrt(raw.size)))

    if side * side != raw.size:
        sys.exit(f"{path}: {raw.size} samples is not square")

    # .hgt row 0 is the NORTHERNMOST row. Flip so row 0 is southernmost,
    # which is what lib/terrain expects (increasing index = increasing north).
    return np.flipud(raw.reshape(side, side)), side


def tile_origin(path):
    """Parse the SW corner lat/lon out of an SRTM filename like N46E008."""
    name = path.split("/")[-1].split(".")[0].upper()

    try:
        lat = int(name[1:3]) * (1 if name[0] == "N" else -1)
        lon = int(name[4:7]) * (1 if name[3] == "E" else -1)
        return lat, lon

    except (ValueError, IndexError):
        sys.exit(f"cannot parse lat/lon from filename '{name}', "
                 "pass --origin explicitly")


def fill_voids(grid):
    """SRTM voids are -32768. Replace with the median of valid samples."""
    bad = grid == SRTM_VOID

    if not bad.any():
        return grid, 0

    good = grid[~bad]

    if good.size == 0:
        sys.exit("patch is entirely void")

    grid = grid.copy()
    grid[bad] = int(np.median(good))
    return grid, int(bad.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hgt", help="input .hgt tile")
    ap.add_argument("-o", "--out", default="terrain.pxtm")
    ap.add_argument("--centre", nargs=2, type=float, metavar=("LAT", "LON"),
                    help="centre of the patch (default: tile centre)")
    ap.add_argument("--span-km", type=float, default=12.0,
                    help="patch width and height in km (default 12)")
    ap.add_argument("--spacing", type=float, default=240.0,
                    help="output sample spacing in metres (default 240, "
                         "which keeps a 12 km patch inside FMUv6 RAM)")
    ap.add_argument("--datum", choices=("min", "mean", "centre", "none"),
                    default="mean",
                    help="elevation subtracted from every sample so the map "
                         "sits near zero (default mean)")
    args = ap.parse_args()

    grid, side = load_hgt(args.hgt)
    tile_lat, tile_lon = tile_origin(args.hgt)

    # SRTM tiles span exactly 1 degree with a 1-sample overlap at each edge.
    deg_per_sample = 1.0 / (side - 1)
    src_spacing_m = 111320.0 * deg_per_sample     # north-south, near enough

    centre_lat, centre_lon = args.centre if args.centre else (tile_lat + 0.5,
                                                              tile_lon + 0.5)
    m_per_deg_lat = 110574.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(centre_lat))

    # How many source samples the requested span covers, and the stride that
    # gets us to the requested output spacing.
    span_m = args.span_km * 1000.0
    stride = max(1, int(round(args.spacing / src_spacing_m)))
    n_out = max(2, int(round(span_m / (src_spacing_m * stride))))

    # Locate the patch. Row index counts north from the tile's south edge.
    row_c = int(round((centre_lat - tile_lat) / deg_per_sample))
    col_c = int(round((centre_lon - tile_lon) / deg_per_sample))
    half = (n_out * stride) // 2
    r0, c0 = row_c - half, col_c - half

    if r0 < 0 or c0 < 0 or r0 + n_out * stride > side or c0 + n_out * stride > side:
        sys.exit(f"patch at {centre_lat},{centre_lon} span {args.span_km} km "
                 f"runs off tile {args.hgt}")

    patch = grid[r0:r0 + n_out * stride:stride,
                 c0:c0 + n_out * stride:stride].astype(np.int32)
    patch, n_void = fill_voids(patch)

    if args.datum == "min":
        datum = int(patch.min())
    elif args.datum == "mean":
        datum = int(round(patch.mean()))
    elif args.datum == "centre":
        datum = int(patch[n_out // 2, n_out // 2])
    else:
        datum = 0

    out = np.clip(patch - datum, -32767, 32767).astype("<i2")

    nbytes = out.size * 2

    if nbytes > MAX_BYTES:
        sys.exit(f"{nbytes // 1024} KB exceeds the {MAX_BYTES // 1024} KB "
                 f"loader cap. Increase --spacing or reduce --span-km.")

    # Actual ground spacing after striding, and the lat/lon of sample (0,0).
    out_spacing_m = src_spacing_m * stride
    origin_lat = tile_lat + r0 * deg_per_sample
    origin_lon = tile_lon + c0 * deg_per_sample

    header = bytearray(HEADER_BYTES)
    header[0:4] = MAGIC
    struct.pack_into("<HHHH", header, 4, VERSION, 0, n_out, n_out)
    struct.pack_into("<f", header, 12, out_spacing_m)
    struct.pack_into("<dd", header, 16, origin_lat, origin_lon)
    struct.pack_into("<f", header, 32, float(datum))

    with open(args.out, "wb") as f:
        f.write(header)
        f.write(out.tobytes())

    km = n_out * out_spacing_m / 1000.0
    print(f"{args.out}: {n_out}x{n_out} @ {out_spacing_m:.1f} m "
          f"= {km:.1f} x {km:.1f} km, {nbytes / 1024:.0f} KB")
    print(f"  origin  {origin_lat:.6f}, {origin_lon:.6f}  (SW corner)")
    print(f"  datum   {datum} m subtracted; "
          f"relief {out.min()} .. {out.max()} m")

    if n_void:
        print(f"  voids   {n_void} samples filled with median")

    print(f"\ncopy to the FC:  {args.out} -> /fs/microsd/etc/terrain.pxtm")
    print(f"then set SIH_TERR_EN=3")


if __name__ == "__main__":
    main()
