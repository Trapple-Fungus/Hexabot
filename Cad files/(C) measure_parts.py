"""Measure bounding boxes of the Hexabot 3D-print files (binary STL + 3MF).

Prints each part's X/Y/Z extents in mm so gait constants in the Arduino
sketches can be tuned against real geometry instead of guesses.
"""
import struct
import sys
import zipfile
import re
from pathlib import Path

HERE = Path(__file__).parent / "3D print files"


def stl_bbox(path):
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:500]:
        # ASCII STL
        floats = re.findall(rb"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", data)
        xs = [float(v[0]) for v in floats]
        ys = [float(v[1]) for v in floats]
        zs = [float(v[2]) for v in floats]
    else:
        # Binary STL: 80-byte header, uint32 tri count, 50 bytes per tri
        (count,) = struct.unpack_from("<I", data, 80)
        xs, ys, zs = [], [], []
        off = 84
        for _ in range(count):
            # skip normal (12 bytes), read 3 vertices
            vals = struct.unpack_from("<12f", data, off)
            for i in range(3, 12, 3):
                xs.append(vals[i]); ys.append(vals[i + 1]); zs.append(vals[i + 2])
            off += 50
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def threemf_bbox(path):
    verts = []
    with zipfile.ZipFile(path) as z:
        # BambuStudio 3MFs keep the mesh in 3D/Objects/*.model sub-files
        for name in z.namelist():
            if not name.endswith(".model"):
                continue
            xml = z.read(name).decode("utf-8", errors="replace")
            verts += re.findall(r'<vertex\s+x="([^"]+)"\s+y="([^"]+)"\s+z="([^"]+)"', xml)
    xs = [float(v[0]) for v in verts]
    ys = [float(v[1]) for v in verts]
    zs = [float(v[2]) for v in verts]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


for f in sorted(HERE.iterdir()):
    try:
        if f.suffix.lower() == ".stl":
            bb = stl_bbox(f)
        elif f.suffix.lower() == ".3mf":
            bb = threemf_bbox(f)
        else:
            continue
    except Exception as e:
        print(f"{f.name}: ERROR {e}")
        continue
    (x0, x1), (y0, y1), (z0, z1) = bb
    print(f"{f.name}")
    print(f"  X: {x0:9.2f} .. {x1:9.2f}  ({x1-x0:7.2f} mm)")
    print(f"  Y: {y0:9.2f} .. {y1:9.2f}  ({y1-y0:7.2f} mm)")
    print(f"  Z: {z0:9.2f} .. {z1:9.2f}  ({z1-z0:7.2f} mm)")
