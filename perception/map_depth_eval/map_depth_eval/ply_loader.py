from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


PLY_DTYPES: Dict[str, str] = {
    "char": "i1",
    "uchar": "u1",
    "short": "i2",
    "ushort": "u2",
    "int": "i4",
    "uint": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}


def load_ply_vertices(path: str) -> Tuple[np.ndarray, np.ndarray | None]:
    file_path = Path(path)
    with file_path.open("rb") as handle:
        format_name, vertex_count, properties = _parse_header(handle)
        dtype = np.dtype(properties)

        if format_name == "binary_little_endian":
            vertices = np.fromfile(handle, dtype=dtype, count=vertex_count)
        elif format_name == "ascii":
            vertices = np.loadtxt(handle, dtype=dtype, max_rows=vertex_count)
        else:
            raise ValueError(f"Unsupported PLY format: {format_name}")

    xyz = np.column_stack((vertices["x"], vertices["y"], vertices["z"])).astype(np.float32)

    rgb = None
    color_fields = ("red", "green", "blue")
    if all(field in vertices.dtype.names for field in color_fields):
        rgb = np.column_stack(
            (vertices["red"], vertices["green"], vertices["blue"])
        ).astype(np.uint8)

    return xyz, rgb


def _parse_header(handle) -> Tuple[str, int, List[Tuple[str, str]]]:
    first_line = handle.readline().decode("ascii", errors="strict").strip()
    if first_line != "ply":
        raise ValueError("File is not a PLY file")

    format_name = ""
    vertex_count = 0
    in_vertex_element = False
    properties: List[Tuple[str, str]] = []

    while True:
        line = handle.readline().decode("ascii", errors="strict")
        if not line:
            raise ValueError("PLY header ended unexpectedly")

        line = line.strip()
        if not line or line.startswith("comment"):
            continue

        if line == "end_header":
            break

        parts = line.split()
        keyword = parts[0]

        if keyword == "format":
            format_name = parts[1]
            continue

        if keyword == "element":
            element_name = parts[1]
            count = int(parts[2])
            in_vertex_element = element_name == "vertex"
            if in_vertex_element:
                vertex_count = count
                properties = []
            continue

        if keyword == "property" and in_vertex_element:
            if parts[1] == "list":
                raise ValueError("PLY list properties are not supported")
            scalar_type = parts[1]
            property_name = parts[2]
            if scalar_type not in PLY_DTYPES:
                raise ValueError(f"Unsupported PLY property type: {scalar_type}")
            properties.append((property_name, PLY_DTYPES[scalar_type]))

    if not format_name:
        raise ValueError("PLY format was not declared")
    if vertex_count <= 0:
        raise ValueError("PLY file does not contain vertices")
    if not properties:
        raise ValueError("PLY vertex properties were not found")

    return format_name, vertex_count, properties
