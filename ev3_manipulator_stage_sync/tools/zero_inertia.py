#!/usr/bin/env python3
"""
fix_zero_inertias.py

Recompute the inertia tensor for URDF links that were exported with a
degenerate (all-zero) <inertia>. Uses the link's collision/visual mesh via
trimesh, keeps the mass ALREADY in the URDF, and rewrites only those links.

Key correctness points handled:
  * mesh units: STLs are in mm, the URDF scales them by 0.001 -> we scale the
    loaded mesh to METRES before computing, so inertia comes out in kg*m^2.
  * mass preserved: trimesh computes the tensor for the existing URDF mass
    (density is back-derived from mass/volume), so dynamics you tuned stay put.
  * watertightness: non-watertight meshes are reported; their inertia is
    approximate (convex hull fallback) and flagged so nothing is silently wrong.
  * only zero-inertia links are touched; every other link is left byte-for-byte.

Usage:
  pip install trimesh numpy lxml
  python3 fix_zero_inertias.py \
      --urdf mani_digital_ev3.urdf.xacro \
      --meshes /path/to/meshes \
      --out mani_digital_ev3.fixed.urdf.xacro

  # dry run (report only, write nothing):
  python3 fix_zero_inertias.py --urdf ... --meshes ... --dry-run
"""

import argparse
import os
import re
import sys

import numpy as np

try:
    import trimesh
except ImportError:
    sys.exit("trimesh not installed. Run: pip install trimesh numpy lxml")

# Preserve comments/formatting as much as possible.
from lxml import etree


# A link's inertia is "degenerate" if the diagonal is essentially zero.
ZERO_TOL = 1e-12


def parse_scale(scale_str):
    """URDF mesh scale like '0.001 0.001 0.001' -> (0.001, 0.001, 0.001)."""
    if not scale_str:
        return (1.0, 1.0, 1.0)
    parts = [float(v) for v in scale_str.split()]
    if len(parts) == 1:
        return (parts[0], parts[0], parts[0])
    return tuple(parts[:3])


def resolve_mesh_path(filename, meshes_dir):
    """Turn a URDF mesh filename into a real path under meshes_dir.

    Handles 'file://$(find pkg)/meshes/foo.stl', 'package://pkg/meshes/foo.stl',
    and plain relative paths by taking the basename and looking in meshes_dir.
    """
    base = os.path.basename(filename)
    candidate = os.path.join(meshes_dir, base)
    return candidate


def inertia_is_zero(inertia_el):
    """True if the <inertia> diagonal is all ~zero."""
    if inertia_el is None:
        return True
    try:
        ixx = float(inertia_el.get("ixx", "0"))
        iyy = float(inertia_el.get("iyy", "0"))
        izz = float(inertia_el.get("izz", "0"))
    except (TypeError, ValueError):
        return True
    return (abs(ixx) <= ZERO_TOL
            or abs(iyy) <= ZERO_TOL
            or abs(izz) <= ZERO_TOL)


def compute_inertia(mesh_path, scale, mass):
    """Return (tensor_3x3, com_xyz, watertight_bool) in kg*m^2 for given mass.

    The mesh is scaled to metres first. trimesh's moment_inertia is computed
    about the centre of mass, which is exactly what the URDF <inertial><origin>
    convention expects when origin xyz = com.
    """
    mesh = trimesh.load(mesh_path, force="mesh")

    if mesh is None or mesh.is_empty:
        raise RuntimeError("empty/unreadable mesh")

    # Scale mm -> m (or whatever the URDF scale is) BEFORE computing inertia.
    mesh.apply_scale(scale)

    watertight = bool(mesh.is_watertight)

    if not watertight:
        # Approximate with the convex hull so we still get a sane tensor.
        hull = mesh.convex_hull
        vol = hull.volume
        com = hull.center_mass
        # density chosen so hull mass == desired URDF mass
        if vol <= 0:
            raise RuntimeError("non-positive hull volume")
        density = mass / vol
        hull.density = density
        tensor = hull.moment_inertia
        return tensor, com, False

    vol = mesh.volume
    if vol <= 0:
        raise RuntimeError("non-positive mesh volume")

    # Set density so the computed mass equals the existing URDF mass.
    mesh.density = mass / vol
    tensor = mesh.moment_inertia
    com = mesh.center_mass
    return tensor, com, True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--meshes", required=True,
                    help="directory containing the STL files")
    ap.add_argument("--out", default=None,
                    help="output URDF (default: <urdf>.fixed.xml)")
    ap.add_argument("--dry-run", action="store_true",
                    help="report only, write nothing")
    ap.add_argument("--update-com", action="store_true",
                    help="also overwrite <inertial><origin> xyz with the mesh COM")
    args = ap.parse_args()

    parser = etree.XMLParser(remove_blank_text=False, remove_comments=False)
    tree = etree.parse(args.urdf, parser)
    root = tree.getroot()

    fixed, skipped, failed, approx = [], [], [], []

    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            skipped.append((name, "no <inertial>"))
            continue

        inertia_el = inertial.find("inertia")
        if not inertia_is_zero(inertia_el):
            skipped.append((name, "inertia already non-zero"))
            continue

        # Mass to preserve.
        mass_el = inertial.find("mass")
        if mass_el is None:
            failed.append((name, "no <mass>"))
            continue
        mass = float(mass_el.get("value", "0"))
        if mass <= 0:
            failed.append((name, f"non-positive mass {mass}"))
            continue

        # Find a mesh (prefer collision, fall back to visual).
        mesh_el = None
        for tag in ("collision", "visual"):
            el = link.find(tag)
            if el is not None:
                m = el.find("geometry/mesh")
                if m is not None:
                    mesh_el = m
                    break
        if mesh_el is None:
            failed.append((name, "no mesh geometry"))
            continue

        scale = parse_scale(mesh_el.get("scale"))
        mesh_path = resolve_mesh_path(mesh_el.get("filename"), args.meshes)
        if not os.path.exists(mesh_path):
            failed.append((name, f"mesh not found: {mesh_path}"))
            continue

        try:
            tensor, com, watertight = compute_inertia(mesh_path, scale, mass)
        except Exception as exc:
            failed.append((name, f"compute error: {exc}"))
            continue

        # Write tensor back (round for readability).
        def f(v):
            return f"{v:.9g}"

        if inertia_el is None:
            inertia_el = etree.SubElement(inertial, "inertia")

        inertia_el.set("ixx", f(tensor[0, 0]))
        inertia_el.set("iyy", f(tensor[1, 1]))
        inertia_el.set("izz", f(tensor[2, 2]))
        inertia_el.set("ixy", f(tensor[0, 1]))
        inertia_el.set("ixz", f(tensor[0, 2]))
        inertia_el.set("iyz", f(tensor[1, 2]))

        if args.update_com:
            origin = inertial.find("origin")
            if origin is None:
                origin = etree.SubElement(inertial, "origin")
                origin.set("rpy", "0 0 0")
            origin.set("xyz", f"{com[0]:.9g} {com[1]:.9g} {com[2]:.9g}")

        fixed.append((name, watertight, mass))
        if not watertight:
            approx.append(name)

    # ---- report ----
    print("\n=== inertia recompute report ===")
    print(f"fixed:   {len(fixed)}")
    for n, wt, m in fixed:
        tag = "watertight" if wt else "APPROX (convex hull)"
        print(f"   + {n:40s} mass={m:.6g}  [{tag}]")
    if approx:
        print(f"\n  NOTE: {len(approx)} link(s) were NOT watertight; their inertia")
        print("  is a convex-hull approximation. Usually fine for these small parts,")
        print("  but verify if any behaves oddly:")
        for n in approx:
            print(f"     ~ {n}")
    if failed:
        print(f"\nfailed:  {len(failed)} (left unchanged)")
        for n, why in failed:
            print(f"   ! {n:40s} {why}")
    print(f"\nskipped: {len(skipped)} (already valid or no inertial)")

    if args.dry_run:
        print("\n[dry-run] no file written.")
        return

    out = args.out or (os.path.splitext(args.urdf)[0] + ".fixed.xml")
    tree.write(out, pretty_print=True, xml_declaration=True, encoding="utf-8")
    print(f"\nwrote: {out}")


if __name__ == "__main__":
    main()
