#!/usr/bin/env python3
"""Shrink a COLLADA file exported by SolidWorks (one geometry per colour, a <triangles> each with
VERTEX and NORMAL at one offset, nodes without transforms): weld the vertices (the export repeats
them per triangle), recompute the normals (smooth below a crease angle) and write the floats short
(positions to 1 um). Colours (effects, materials) and the scene are kept as they are. The meshes of
urdf/quad_with_dual_arm/gimbalrotor_color.xacro (mesh/*.dae) are made by it, about 1/8 of the export
in size for the same look:

  python3 compact_dae.py <exported.dae> <mesh/name.dae>

--ratio below 1 also decimates each geometry (quadric, pip install fast-simplification), but it breaks
the thin parts of these CAD meshes even at 0.5.

  compact_dae.py <in.dae> <out.dae> [--ratio R] [--min-faces N] [--angle DEG]
"""

import argparse
import re
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

NS = "http://www.collada.org/2005/11/COLLADASchema"
Q = "{%s}" % NS


def fmt(values, decimals):
    text = np.char.mod("%%.%df" % decimals, np.round(values, decimals).ravel())
    text = [re.sub(r"\.?0+$", "", t) if "." in t else t for t in text]
    return " ".join("0" if t in ("-0", "") else t for t in text)


def process(geometry, ratio, min_faces, angle):
    mesh = geometry.find(Q + "mesh")
    sources = {s.get("id"): s for s in mesh.findall(Q + "source")}
    vertices = mesh.find(Q + "vertices")
    pos_id = vertices.find(Q + "input[@semantic='POSITION']").get("source")[1:]
    positions = np.array(sources[pos_id].find(Q + "float_array").text.split(), dtype=float).reshape(-1, 3)
    triangles = mesh.find(Q + "triangles")
    inputs = triangles.findall(Q + "input")
    stride = max(int(i.get("offset")) for i in inputs) + 1
    vertex_offset = int(next(i for i in inputs if i.get("semantic") == "VERTEX").get("offset"))
    p = np.array(triangles.find(Q + "p").text.split(), dtype=np.int64).reshape(-1, stride)[:, vertex_offset]
    faces = p.reshape(-1, 3)
    before = len(faces)

    m = trimesh.Trimesh(positions, faces, process=False)
    m.merge_vertices(digits_vertex=7)  # weld (the export duplicates them per triangle)
    m.update_faces(m.nondegenerate_faces())
    m.remove_unreferenced_vertices()
    target = max(min_faces, int(len(m.faces) * ratio))
    if target < len(m.faces):
        import fast_simplification
        v, f = fast_simplification.simplify(m.vertices, m.faces, target_reduction=1.0 - target / len(m.faces))
        m = trimesh.Trimesh(v, f, process=False)
    m = trimesh.graph.smooth_shade(m, angle=np.radians(angle))
    v, f, n = m.vertices, m.faces, m.vertex_normals

    gid = geometry.get("id")
    for s in list(mesh.findall(Q + "source")):
        mesh.remove(s)
    for i, (name, data, decimals) in enumerate((("positions", v, 6), ("normals", n, 3))):
        source = ET.Element(Q + "source", id="%s-%s" % (gid, name))
        array = ET.SubElement(source, Q + "float_array", id="%s-%s-array" % (gid, name), count=str(data.size))
        array.text = fmt(data, decimals)
        common = ET.SubElement(source, Q + "technique_common")
        accessor = ET.SubElement(common, Q + "accessor", source="#%s-%s-array" % (gid, name),
                                 count=str(len(data)), stride="3")
        for axis in "XYZ":
            ET.SubElement(accessor, Q + "param", name=axis, type="float")
        mesh.insert(i, source)
    vertices.find(Q + "input[@semantic='POSITION']").set("source", "#%s-positions" % gid)
    for i in inputs:
        triangles.remove(i)
    p_elem = triangles.find(Q + "p")
    triangles.remove(p_elem)
    ET.SubElement(triangles, Q + "input", semantic="VERTEX", source="#%s-vertices" % gid, offset="0")
    ET.SubElement(triangles, Q + "input", semantic="NORMAL", source="#%s-normals" % gid, offset="0")
    p_new = ET.SubElement(triangles, Q + "p")
    p_new.text = " ".join(map(str, f.ravel()))
    triangles.set("count", str(len(f)))
    return before, len(f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--ratio", type=float, default=1.0, help="share of the triangles kept per geometry")
    parser.add_argument("--min-faces", type=int, default=200, help="no decimation below this per geometry")
    parser.add_argument("--angle", type=float, default=30.0, help="[deg] crease angle of the normals")
    args = parser.parse_args()
    ET.register_namespace("", NS)
    tree = ET.parse(args.input)
    total = [0, 0]
    for geometry in tree.getroot().iter(Q + "geometry"):
        b, a = process(geometry, args.ratio, args.min_faces, args.angle)
        total[0] += b
        total[1] += a
    ET.indent(tree, "  ")
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print("%s: %d -> %d triangles" % (args.input.split("/")[-1], total[0], total[1]))


if __name__ == "__main__":
    main()
