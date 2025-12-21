#!/usr/bin/env python3
"""
Patch wheel/caster friction into an SDF file and remove invalid link-level <surface> tags.

Usage:
  python3 patch_friction.py INPUT.sdf OUTPUT.sdf

Pipeline:
  ros2 run xacro xacro k9.urdf.xacro > /tmp/k9.urdf
  gz sdf -p /tmp/k9.urdf > /tmp/k9_raw.sdf
  python3 patch_friction.py /tmp/k9_raw.sdf /tmp/k9.sdf
"""

import sys
import xml.etree.ElementTree as ET
from typing import Dict, Tuple, Optional


# Friction values: (mu, mu2)
FRICTION_MAP: Dict[str, Tuple[float, float]] = {
    "drivewhl_l_link": (1.1, 0.9),
    "drivewhl_r_link": (1.1, 0.9),
    "caster_wheel_link": (0.3, 0.15),
}

TARGET_MODEL_NAME: Optional[str] = "k9_robot"  # set to None to patch all models


def find_first(parent: ET.Element, tag: str) -> Optional[ET.Element]:
    for child in parent:
        if child.tag == tag:
            return child
    return None


def find_link(model: ET.Element, name: str) -> Optional[ET.Element]:
    for link in model.findall("link"):
        if link.get("name") == name:
            return link
    return None


def find_collision(link: ET.Element) -> Optional[ET.Element]:
    # Prefer a collision whose name ends with "_collision"
    for col in link.findall("collision"):
        if col.get("name", "").endswith("_collision"):
            return col
    # Fallback: first collision
    return find_first(link, "collision")


def remove_link_level_surfaces(link: ET.Element) -> int:
    """Remove invalid <surface> elements directly under <link>."""
    removed = 0
    for child in list(link):
        if child.tag == "surface":
            link.remove(child)
            removed += 1
    return removed


def ensure_friction(collision: ET.Element, mu: float, mu2: float) -> None:
    surface = find_first(collision, "surface")
    if surface is None:
        surface = ET.SubElement(collision, "surface")

    friction = find_first(surface, "friction")
    if friction is None:
        friction = ET.SubElement(surface, "friction")

    ode = find_first(friction, "ode")
    if ode is None:
        ode = ET.SubElement(friction, "ode")

    mu_el = find_first(ode, "mu")
    if mu_el is None:
        mu_el = ET.SubElement(ode, "mu")
    mu_el.text = f"{mu:.6g}"

    mu2_el = find_first(ode, "mu2")
    if mu2_el is None:
        mu2_el = ET.SubElement(ode, "mu2")
    mu2_el.text = f"{mu2:.6g}"


def main() -> int:
    if len(sys.argv) != 3:
        print("Usage: patch_friction.py INPUT.sdf OUTPUT.sdf", file=sys.stderr)
        return 2

    infile, outfile = sys.argv[1], sys.argv[2]

    tree = ET.parse(infile)
    root = tree.getroot()

    if root.tag != "sdf":
        print("[ERROR] Root element is not <sdf>", file=sys.stderr)
        return 1

    # Collect models (either direct or inside <world>)
    models = root.findall("model")
    for world in root.findall("world"):
        models.extend(world.findall("model"))

    patched_links = 0
    removed_surfaces = 0

    for model in models:
        model_name = model.get("name")
        if TARGET_MODEL_NAME and model_name != TARGET_MODEL_NAME:
            continue

        for link_name, (mu, mu2) in FRICTION_MAP.items():
            link = find_link(model, link_name)
            if link is None:
                continue

            # 1) Remove invalid link-level <surface>
            removed_surfaces += remove_link_level_surfaces(link)

            # 2) Ensure collision-level friction
            collision = find_collision(link)
            if collision is None:
                print(f"[WARN] No collision found for link '{link_name}'", file=sys.stderr)
                continue

            ensure_friction(collision, mu, mu2)
            patched_links += 1

    ET.indent(tree, space="  ", level=0)
    tree.write(outfile, encoding="utf-8", xml_declaration=True)

    print(
        f"[OK] Patched {patched_links} link(s), removed {removed_surfaces} invalid <surface> tag(s).",
        file=sys.stderr,
    )
    print(f"[OK] Wrote cleaned SDF: {outfile}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
