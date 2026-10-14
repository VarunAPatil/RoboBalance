"""Convert the corrected Wall-E URDF into a static, geometry-only MJCF preview.

The source export has invalid/tiny per-link inertias and incomplete mass data.
This converter deliberately does not produce a dynamic robot model.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PROJECT / "Wall-E-URDF/urdf/assembly_1.urdf"
MODEL_PATH = PROJECT / "simulation/models/wall_e_preview.xml"
REPORT_PATH = PROJECT / "simulation/models/wall_e_preview_audit.json"
STL_DTYPE = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])


def numbers(element: ET.Element | None, attribute: str, default: tuple[float, ...], length: int) -> np.ndarray:
    if element is None or element.get(attribute) is None:
        result = np.array(default, dtype=float)
    else:
        try:
            result = np.array([float(value) for value in element.get(attribute).split()], dtype=float)
        except ValueError as exc:
            raise ValueError(f"Invalid {attribute} on {element.tag}") from exc
    if result.size != length or not np.isfinite(result).all():
        raise ValueError(f"Expected {length} finite {attribute} values on {element.tag if element is not None else 'origin'}")
    return result


def rpy_rotation(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def origin_transform(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rpy_rotation(numbers(element, "rpy", (0, 0, 0), 3))
    transform[:3, 3] = numbers(element, "xyz", (0, 0, 0), 3)
    return transform


def quat_wxyz(rotation: np.ndarray) -> np.ndarray:
    # Quaternion order required by MJCF: w, x, y, z.
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        q = np.array([0.25 * scale, (rotation[2, 1] - rotation[1, 2]) / scale,
                      (rotation[0, 2] - rotation[2, 0]) / scale,
                      (rotation[1, 0] - rotation[0, 1]) / scale])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            q = np.array([(rotation[2, 1] - rotation[1, 2]) / scale, 0.25 * scale,
                          (rotation[0, 1] + rotation[1, 0]) / scale,
                          (rotation[0, 2] + rotation[2, 0]) / scale])
        elif index == 1:
            scale = math.sqrt(1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            q = np.array([(rotation[0, 2] - rotation[2, 0]) / scale,
                          (rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                          (rotation[1, 2] + rotation[2, 1]) / scale])
        else:
            scale = math.sqrt(1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            q = np.array([(rotation[1, 0] - rotation[0, 1]) / scale,
                          (rotation[0, 2] + rotation[2, 0]) / scale,
                          (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale])
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return q


def fmt(values: np.ndarray | list[float]) -> str:
    return " ".join(f"{float(value):.10g}" for value in values)


def read_stl(path: Path) -> np.ndarray:
    content = path.read_bytes()
    if len(content) < 84:
        raise ValueError(f"STL too short: {path}")
    count = struct.unpack_from("<I", content, 80)[0]
    if count < 4 or len(content) != 84 + 50 * count:
        raise ValueError(f"Invalid binary STL facets/length: {path}")
    vertices = np.frombuffer(content, dtype=STL_DTYPE, offset=84, count=count)["vertices"]
    if not np.isfinite(vertices).all():
        raise ValueError(f"Non-finite mesh coordinates: {path}")
    return vertices.reshape(-1, 3)


def signed_volume(vertices: np.ndarray) -> float:
    faces = vertices.reshape(-1, 3, 3).astype(float)
    return float(np.einsum("ij,ij->", faces[:, 0], np.cross(faces[:, 1], faces[:, 2])) / 6.0)


def resolve_source(requested: Path | None) -> Path:
    if requested is not None:
        source = requested.resolve()
    elif DEFAULT_SOURCE.is_file():
        source = DEFAULT_SOURCE
    else:
        raise FileNotFoundError(f"Expected corrected assembly source: {DEFAULT_SOURCE}")
    if not source.is_file() or not (source.parent.parent / "meshes").is_dir():
        raise FileNotFoundError(f"URDF or mesh directory missing: {source}")
    if (source.name != "assembly_1.urdf" or source.parent.name != "urdf"
            or source.parent.parent.name != "Wall-E-URDF"):
        raise ValueError(f"Refusing older/unexpected source: {source}")
    print(f"Selected assembly source: {source}")
    return source


def load_urdf(source: Path) -> tuple[dict, dict, str, dict, float]:
    tree = ET.parse(source)
    robot = tree.getroot()
    if robot.tag != "robot":
        raise ValueError("URDF root must be <robot>")
    links = {}
    for link in robot.findall("link"):
        name = link.get("name")
        if not name or name in links:
            raise ValueError(f"Missing/duplicate link name: {name}")
        links[name] = link
    children = defaultdict(list)
    child_names = set()
    counts = Counter()
    joints = {}
    for joint in robot.findall("joint"):
        name, kind = joint.get("name"), joint.get("type")
        parent = joint.find("parent")
        child = joint.find("child")
        if not name or name in joints or kind not in ("fixed", "continuous"):
            raise ValueError(f"Unsupported/duplicate joint {name}: {kind}")
        if parent is None or child is None or parent.get("link") not in links or child.get("link") not in links:
            raise ValueError(f"Broken parent/child reference: {name}")
        child_name = child.get("link")
        if child_name in child_names:
            raise ValueError(f"Link has multiple parents: {child_name}")
        child_names.add(child_name)
        axis = numbers(joint.find("axis"), "xyz", (1, 0, 0), 3)
        if np.linalg.norm(axis) < 1e-12:
            raise ValueError(f"Zero joint axis: {name}")
        origin_transform(joint.find("origin"))
        children[parent.get("link")].append((joint, child_name))
        joints[name] = joint
        counts[kind] += 1
    roots = set(links) - child_names
    if len(roots) != 1 or len(joints) != len(links) - 1:
        raise ValueError(f"URDF must be a single tree; roots={roots}")
    root_name = roots.pop()
    transforms = {}
    groups = {}
    hinges = {}

    def visit(link_name: str, transform: np.ndarray, group: str) -> None:
        if link_name in transforms:
            raise ValueError(f"URDF cycle/repeated link: {link_name}")
        transforms[link_name] = transform
        groups[link_name] = group
        for joint, child_name in children[link_name]:
            child_transform = transform @ origin_transform(joint.find("origin"))
            next_group = group
            if joint.get("type") == "continuous":
                next_group = joint.get("name")
                local_axis = numbers(joint.find("axis"), "xyz", (1, 0, 0), 3)
                local_axis /= np.linalg.norm(local_axis)
                hinges[next_group] = {"child": child_name, "parent": link_name,
                                      "frame": child_transform, "axis_local": local_axis,
                                      "axis_world": child_transform[:3, :3] @ local_axis}
            visit(child_name, child_transform, next_group)

    visit(root_name, np.eye(4), "chassis")
    if len(transforms) != len(links) or len(hinges) != 2 or counts["continuous"] != 2 or counts["fixed"] != len(links) - 3:
        raise ValueError(f"Unexpected assembly structure: {len(transforms)} links, {counts}, {len(hinges)} hinges")
    exported_mass = 0.0
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is not None:
            mass = inertial.find("mass")
            inertia = inertial.find("inertia")
            if mass is None or inertia is None:
                raise ValueError(f"Incomplete inertial at {name}")
            value = numbers(mass, "value", (0,), 1)[0]
            if value < 0:
                raise ValueError(f"Negative mass at {name}")
            exported_mass += value
            for component in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
                numbers(inertia, component, (0,), 1)
            origin_transform(inertial.find("origin"))
    return links, transforms, root_name, {"groups": groups, "hinges": hinges, "joint_counts": counts}, exported_mass


def camera_quat(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    back = position - target
    back /= np.linalg.norm(back)
    right = np.cross(np.array([0.0, 0.0, 1.0]), back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    return quat_wxyz(np.column_stack((right, up, back)))


def fallback_rgba(name: str) -> str:
    key = name.lower()
    if "battery18650" in key:
        return "0.30 0.50 0.71 1"
    if "batterycasespring" in key or "screw" in key:
        return "0.72 0.74 0.76 1"
    if "batterycaseplastic" in key:
        return "0.22 0.24 0.28 1"
    if "pcb" in key:
        return "0.08 0.33 0.22 1"
    if "motor" in key or "shaft" in key:
        return "0.48 0.54 0.60 1"
    return "0.66 0.69 0.73 1"


def convert(source: Path, output: Path, report_path: Path) -> dict:
    links, transforms, root_name, topology, exported_mass = load_urdf(source)
    mesh_root = (source.parent.parent / "meshes").resolve()
    group_frames = {"chassis": np.eye(4)}
    group_frames.update({key: value["frame"] for key, value in topology["hinges"].items()})
    visuals = []
    vertices_cache = {}
    source_colors = 0
    for link_name, link in links.items():
        for index, visual in enumerate(link.findall("visual")):
            mesh = visual.find("./geometry/mesh")
            if mesh is None:
                raise ValueError(f"Unsupported non-mesh visual: {link_name}")
            uri = mesh.get("filename", "")
            prefix = "package://assembly_1/meshes/"
            if not uri.startswith(prefix):
                raise ValueError(f"Unexpected mesh URI: {uri}")
            path = (mesh_root / uri[len(prefix):]).resolve()
            try:
                path.relative_to(mesh_root)
            except ValueError as exc:
                raise ValueError(f"Mesh path escapes package: {uri}") from exc
            if not path.is_file():
                raise FileNotFoundError(path)
            scale = numbers(mesh, "scale", (1, 1, 1), 3)
            if np.any(scale <= 0):
                raise ValueError(f"Non-positive mesh scale: {uri}")
            if path not in vertices_cache:
                vertices_cache[path] = read_stl(path)
            color = visual.find("./material/color")
            rgba = fallback_rgba(link_name)
            if color is not None:
                values = numbers(color, "rgba", (1, 1, 1, 1), 4)
                if np.any(values < 0) or np.any(values > 1):
                    raise ValueError(f"Invalid RGBA on {link_name}")
                rgba = fmt(values)
                source_colors += 1
            group = topology["groups"][link_name]
            transform = transforms[link_name] @ origin_transform(visual.find("origin"))
            vertices_world = (vertices_cache[path].astype(float) * scale) @ transform[:3, :3].T + transform[:3, 3]
            visuals.append({"link": link_name, "index": index, "path": path, "scale": tuple(scale),
                            "rgba": rgba, "group": group, "transform": transform,
                            "min": vertices_world.min(axis=0), "max": vertices_world.max(axis=0)})
    if not visuals:
        raise ValueError("Assembly has no visual meshes")
    bounds_min = np.min([item["min"] for item in visuals], axis=0)
    bounds_max = np.max([item["max"] for item in visuals], axis=0)
    center_xy = (bounds_min[:2] + bounds_max[:2]) / 2
    display = np.array([-center_xy[0], -center_xy[1], 0.003 - bounds_min[2]])
    final_min, final_max = bounds_min + display, bounds_max + display

    model = ET.Element("mujoco", model="wall_e_geometry_only_preview")
    ET.SubElement(model, "compiler", angle="radian", inertiafromgeom="false")
    ET.SubElement(model, "option", gravity="0 0 0")
    visual_settings = ET.SubElement(model, "visual")
    ET.SubElement(visual_settings, "global", offwidth="1280", offheight="960", azimuth="135", elevation="25")
    ET.SubElement(visual_settings, "rgba", haze="0.78 0.83 0.88 1")
    assets = ET.SubElement(model, "asset")
    ET.SubElement(assets, "texture", name="sky", type="skybox", builtin="gradient",
                  rgb1="0.72 0.80 0.87", rgb2="0.95 0.97 0.99", width="512", height="512")
    ET.SubElement(assets, "texture", name="floor_grid", type="2d", builtin="checker",
                  rgb1="0.82 0.84 0.86", rgb2="0.88 0.90 0.92", width="256", height="256")
    ET.SubElement(assets, "material", name="floor_mat", texture="floor_grid", texrepeat="12 12", reflectance="0")
    mesh_keys = sorted({(item["path"], item["scale"]) for item in visuals}, key=lambda key: (str(key[0]), key[1]))
    mesh_names = {}
    for index, (path, scale) in enumerate(mesh_keys):
        name = f"asset_{index:04d}"
        mesh_names[(path, scale)] = name
        relative = os.path.relpath(path, output.parent).replace(os.sep, "/")
        ET.SubElement(assets, "mesh", name=name, file=relative, scale=fmt(scale))
    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "light", name="key", pos="0.2 -0.25 0.5", dir="-0.3 0.5 -1",
                  directional="true", diffuse="0.9 0.9 0.9")
    ET.SubElement(world, "light", name="fill", pos="-0.2 0.2 0.4", dir="0.4 -0.5 -1",
                  directional="true", diffuse="0.5 0.55 0.6")
    ET.SubElement(world, "geom", name="visual_floor", type="plane", pos="0 0 0", size="0.55 0.55 0.01",
                  material="floor_mat", contype="0", conaffinity="0")
    camera_specs = {
        "three_quarter": (np.array([0.25, -0.30, 0.18]), np.array([0, 0, 0.055])),
        "battery_side": (np.array([-0.23, 0.28, 0.18]), np.array([0, 0, 0.052])),
    }
    for name, (position, target) in camera_specs.items():
        ET.SubElement(world, "camera", name=name, pos=fmt(position), quat=fmt(camera_quat(position, target)))
    chassis = ET.SubElement(world, "body", name="chassis_static", pos=fmt(display))
    bodies = {"chassis": chassis}
    hinge_order = sorted(topology["hinges"], key=lambda key: topology["hinges"][key]["frame"][0, 3])
    for side, key in zip(("left", "right"), hinge_order):
        frame = group_frames[key]
        bodies[key] = ET.SubElement(chassis, "body", name=f"wheel_{side}_static",
                                    pos=fmt(frame[:3, 3]), quat=fmt(quat_wxyz(frame[:3, :3])))
    for item in visuals:
        group = item["group"]
        relative = np.linalg.inv(group_frames[group]) @ item["transform"]
        ET.SubElement(bodies[group], "geom", name=f"visual_{item['link']}_{item['index']}", type="mesh",
                      mesh=mesh_names[(item["path"], item["scale"])], pos=fmt(relative[:3, 3]),
                      quat=fmt(quat_wxyz(relative[:3, :3])), rgba=item["rgba"],
                      contype="0", conaffinity="0")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(model, space="  ")
    ET.ElementTree(model).write(output, encoding="unicode", xml_declaration=True)

    special = {}
    for label, names in {
        "battery_cells_mesh": ["battery18650__none_2"],
        "battery_holder_mesh": ["batterycaseplastic__fillet1"],
        "battery_springs_mesh": ["batterycasespring__cut_extrude2"],
        "main_pcb_mesh": ["sra_board_unsaved_export_ok3bnj_pcb"],
        "daughterboard_mesh": ["body___pcb"],
        "screws": [f"screw_m3_75mm{suffix}" for suffix in ("", "_1", "_2", "_3")],
        "motor_shells": ["tt_motor_90_1_solid1", "tt_motor_90_1_solid1_1"],
        "motor_cores": ["mini_motor_1_solid1", "mini_motor_1_solid1_1"],
        "motor_shafts": ["output_shaft_2_solid1", "output_shaft_2_solid1_1"],
        "rims": ["rim", "rim_1"],
        "tires": ["tyre", "tyre_1"],
    }.items():
        found = [item for item in visuals if item["link"] in names]
        if len(found) != len(names):
            raise ValueError(f"Missing/duplicated key visuals for {label}: {len(found)} versus {len(names)}")
        special[label] = [{"link": item["link"], "mesh": item["path"].name,
                           "group": item["group"]} for item in found]
    battery_volume = sum(abs(signed_volume(vertices_cache[next(item["path"] for item in visuals if item["link"] == name)]))
                         for name in ("battery18650__none_2", "batterycaseplastic__fillet1", "batterycasespring__cut_extrude2"))
    report = {
        "mode": "static geometry-only; no joints, inertials, actuation, contact dynamics, or RL",
        "source_urdf": str(source.relative_to(PROJECT)),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_links": len(links), "source_joints": dict(topology["joint_counts"]),
        "root_link": root_name, "source_exported_mass_kg_unreliable": exported_mass,
        "visual_instances": len(visuals), "unique_mesh_assets": len(mesh_keys),
        "visuals_with_source_rgba": source_colors,
        "group_visual_counts": dict(Counter(item["group"] for item in visuals)),
        "source_bounds_m": {"min": bounds_min.tolist(), "max": bounds_max.tolist()},
        "display_translation_m": display.tolist(),
        "display_bounds_m": {"min": final_min.tolist(), "max": final_max.tolist()},
        "display_transform_note": "Only one global translation: center XY and place lowest geometry 3 mm over visual floor. No individual mesh recentering or compensation.",
        "hinges_inspection_only": {key: {"child": value["child"], "parent": value["parent"],
                                          "origin_source_m": value["frame"][:3, 3].tolist(),
                                          "axis_in_child_frame": value["axis_local"].tolist(),
                                          "axis_in_source_world": value["axis_world"].tolist()}
                                   for key, value in topology["hinges"].items()},
        "key_visuals": special,
        "mass_targets_not_applied_to_static_scene": {
            "battery_cells_holder_springs_shared_density_group_kg": 0.153,
            "battery_group_complete_signed_mesh_volume_m3_abs_sum": battery_volume,
            "battery_group_effective_uniform_density_kg_m3": 0.153 / battery_volume,
            "each_screw_kg": 0.0045, "each_complete_wheel_kg": 0.040,
            "each_complete_motor_including_shaft_kg": 0.029,
            "populated_pcb_target_kg": 0.030,
        },
        "remaining_physical_model_limits": [
            "Source tiny/invalid per-component inertias omitted; direct URDF import previously failed at 0603_resistor_004_2.",
            "Many tiny electronics meshes are non-watertight/disconnected; no complete trustworthy aggregate inertia yet.",
            "The 30 g target covers the populated main PCB and mounted components, but this corrected export has not had its mass/COM audit recomputed.",
            "Complete assembled mass, wheel-motion sensing, motor response, battery voltage, contact/friction and sensor timing are not yet characterized.",
        ],
        "dependency_versions": {
            "python": sys.version.split()[0], "numpy": np.__version__,
            "mujoco": importlib.metadata.version("mujoco"),
            "Pillow": importlib.metadata.version("Pillow"),
        },
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {output} with {len(visuals)} visual geoms, {len(mesh_keys)} assets, 3 fixed bodies")
    print(f"Wrote {report_path}")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, help="Explicit weighted URDF path if package was renamed")
    args = parser.parse_args()
    source = resolve_source(args.source)
    convert(source, MODEL_PATH, REPORT_PATH)


if __name__ == "__main__":
    main()
