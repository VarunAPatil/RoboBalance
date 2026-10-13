"""Audit Wall-E mass and centre of mass without inventing dynamic inertias.

The visual MJCF remains a static assembly. This script uses the URDF's other
per-instance masses/centres (as confirmed by the user), corrects the complete
battery group and four screws from their closed meshes, restores repeated
populated-PCB instances, and compares that reconstruction to the 30 g target.
"""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np

from simulation.convert_urdf import (
    PROJECT,
    REPORT_PATH,
    MODEL_PATH,
    load_urdf,
    numbers,
    origin_transform,
    read_stl,
    resolve_source,
)
from simulation.repeated_solids import reconstruct_pcb_repeats


AUDIT_PATH = PROJECT / "simulation/models/wall_e_mass_audit.json"
MARKERS_PATH = PROJECT / "simulation/models/wall_e_com_inspection.xml"
BOARD_ROOT = "sra_board_unsaved_export_ok3bnj_pcb"
BATTERY_ROOT = "batterycaseplastic__fillet1"
BATTERY_SOLIDS = (
    "battery18650__none_2",
    BATTERY_ROOT,
    "batterycasespring__cut_extrude2",
)
SCREWS = tuple("screw_m3_75mm" + suffix for suffix in ("", "_1", "_2", "_3"))
MOTOR_LINKS = {
    "left": ("tt_motor_90_1_solid1", "mini_motor_1_solid1_1", "output_shaft_2_solid1_1"),
    "right": ("tt_motor_90_1_solid1_1", "mini_motor_1_solid1", "output_shaft_2_solid1"),
}
BATTERY_MASS_KG = 0.153
SCREW_MASS_KG = 0.0045
PCB_TARGET_KG = 0.030
STRUCTURAL_PARTS = ("part_1", "part_1_1", "part_2")
STRUCTURAL_PART_MASSES_KG = {"part_1": 0.032, "part_1_1": 0.00442, "part_2": 0.00442}


def solid_properties(vertices: np.ndarray) -> dict:
    """Integrate the actual closed triangular solid, never its convex hull.

    Returns volume, volume centroid, and central inertia at unit density.
    STL coordinates and returned properties use m, m3, and m5 respectively.
    """
    triangles = np.asarray(vertices, dtype=np.float64).reshape(-1, 3, 3)
    flat = triangles.reshape(-1, 3)
    unique, inverse = np.unique(flat, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    if len(unique) < 4 or len(faces) < 4:
        raise ValueError("Mesh has too few faces or vertices for a solid")
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    if np.any(np.linalg.norm(cross, axis=1) <= 1e-18):
        raise ValueError("Mesh contains a degenerate triangle")

    directed = np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    edges = np.sort(directed, axis=1)
    _, edge_index = np.unique(edges, axis=0, return_inverse=True)
    incidence = np.bincount(edge_index)
    orientation = np.bincount(
        edge_index,
        weights=np.where(directed[:, 0] < directed[:, 1], 1, -1),
    )
    if np.any(incidence != 2) or np.any(orientation != 0):
        raise ValueError("Mesh is not watertight with consistent face winding")

    # Count separate shells so repeated cells or springs are integrated once
    # each, and reject reversed internal shells whose cavity handling is unclear.
    parent = np.arange(len(unique))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    for a, b, c in faces:
        ra, rb, rc = find(int(a)), find(int(b)), find(int(c))
        parent[rb] = ra
        parent[rc] = ra
    shell_ids = np.array([find(int(a)) for a in faces[:, 0]], dtype=int)
    _, shell_index = np.unique(shell_ids, return_inverse=True)

    # Shift the tetrahedral reference near the mesh to reduce cancellation.
    reference = (unique.min(axis=0) + unique.max(axis=0)) / 2
    a, b, c = (triangles[:, i] - reference for i in range(3))
    tetra_volume = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6
    shell_volumes = np.bincount(shell_index, weights=tetra_volume)
    if np.any(shell_volumes <= 1e-12):
        raise ValueError("Mesh has a zero-volume or reversed connected shell")
    volume = float(shell_volumes.sum())
    summed_vertices = a + b + c
    first_moment = np.einsum("i,ij->j", tetra_volume / 4, summed_vertices)
    centroid_relative = first_moment / volume
    second_moment = np.zeros((3, 3))
    for vectors in (summed_vertices, a, b, c):
        second_moment += np.einsum("i,ij,ik->jk", tetra_volume / 20, vectors, vectors)
    central_second = second_moment - volume * np.outer(centroid_relative, centroid_relative)
    central_second = (central_second + central_second.T) / 2
    inertia_at_unit_density = np.trace(central_second) * np.eye(3) - central_second
    inertia_at_unit_density = (inertia_at_unit_density + inertia_at_unit_density.T) / 2
    validate_inertia(inertia_at_unit_density)
    return {
        "volume_m3": volume,
        "centroid_m": centroid_relative + reference,
        "inertia_unit_density_m5": inertia_at_unit_density,
        "closed_shells": len(shell_volumes),
        "shell_volumes_m3": shell_volumes.tolist(),
        "bounds_m": [unique.min(axis=0).tolist(), unique.max(axis=0).tolist()],
        "facets": len(triangles),
    }


def validate_inertia(inertia: np.ndarray) -> None:
    if not np.isfinite(inertia).all() or not np.allclose(inertia, inertia.T, atol=1e-15, rtol=0):
        raise ValueError("Non-finite or asymmetric inertia tensor")
    principal = np.linalg.eigvalsh(inertia)
    if principal[0] <= 0 or principal[0] + principal[1] < principal[2] * (1 - 1e-8):
        raise ValueError(f"Invalid physical principal inertias: {principal}")


def subtree(root: str, children: dict[str, list[str]]) -> list[str]:
    names = [root]
    for name in names:
        names.extend(children.get(name, []))
    return names


def weighted_center(records: dict[str, dict], names: list[str] | tuple[str, ...]) -> tuple[float, np.ndarray]:
    mass = sum(records[name]["mass_kg"] for name in names)
    if mass <= 0:
        raise ValueError("Cannot find COM of a zero-mass group")
    center = sum((records[name]["mass_kg"] * records[name]["com_source_m"] for name in names),
                 np.zeros(3)) / mass
    return mass, center


def mesh_link_properties(name: str, links: dict, transforms: dict, source: Path, cache: dict) -> dict:
    visuals = links[name].findall("visual")
    if len(visuals) != 1:
        raise ValueError(f"Expected one visual for weighed solid {name}: {len(visuals)}")
    visual = visuals[0]
    mesh = visual.find("./geometry/mesh")
    if mesh is None:
        raise ValueError(f"Missing visual mesh at {name}")
    prefix = "package://assembly_1/meshes/"
    uri = mesh.get("filename", "")
    if not uri.startswith(prefix):
        raise ValueError(f"Unexpected mesh URI at {name}: {uri}")
    mesh_root = (source.parent.parent / "meshes").resolve()
    path = (mesh_root / uri[len(prefix):]).resolve()
    path.relative_to(mesh_root)
    scale = numbers(mesh, "scale", (1, 1, 1), 3)
    if np.any(scale <= 0):
        raise ValueError(f"Invalid mesh scale at {name}")
    cache_key = (path, tuple(scale))
    if cache_key not in cache:
        if not path.is_file():
            raise FileNotFoundError(path)
        properties = solid_properties(read_stl(path) * scale)
        cache[cache_key] = properties
    else:
        properties = cache[cache_key]
    pose = transforms[name] @ origin_transform(visual.find("origin"))
    centroid = pose[:3, :3] @ properties["centroid_m"] + pose[:3, 3]
    return {
        "link": name,
        "mesh": path.name,
        "mesh_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "scale": scale.tolist(),
        "volume_m3": properties["volume_m3"],
        "closed_shells": properties["closed_shells"],
        "shell_volumes_m3": properties["shell_volumes_m3"],
        "facets": properties["facets"],
        "com_source_m": centroid,
        "inertia_unit_density_source_m5": pose[:3, :3] @ properties["inertia_unit_density_m5"] @ pose[:3, :3].T,
        "bounds_local_m": properties["bounds_m"],
    }


def aggregate_mesh_group(items: list[dict], masses: list[float]) -> dict:
    if not items or len(items) != len(masses) or not np.isfinite(masses).all() or any(mass <= 0 for mass in masses):
        raise ValueError("Each closed mesh must have one finite positive assigned mass")
    total = float(sum(masses))
    center = sum((mass * item["com_source_m"] for mass, item in zip(masses, items)), np.zeros(3)) / total
    inertia = np.zeros((3, 3))
    for item, mass in zip(items, masses):
        density = mass / item["volume_m3"]
        displacement = item["com_source_m"] - center
        inertia += density * item["inertia_unit_density_source_m5"]
        inertia += mass * (np.dot(displacement, displacement) * np.eye(3) - np.outer(displacement, displacement))
    inertia = (inertia + inertia.T) / 2
    validate_inertia(inertia)
    return {
        "mass_kg": total,
        "com_source_m": center.tolist(),
        "inertia_about_com_source_kg_m2": inertia.tolist(),
        "principal_inertias_kg_m2": np.linalg.eigvalsh(inertia).tolist(),
    }


def json_item(item: dict, mass: float) -> dict:
    return {
        "link": item["link"], "mesh": item["mesh"], "mesh_sha256": item["mesh_sha256"],
        "scale": item["scale"], "closed_shells": item["closed_shells"],
        "shell_volumes_m3": item["shell_volumes_m3"], "facets": item["facets"],
        "bounds_local_m": item["bounds_local_m"], "volume_m3": item["volume_m3"],
        "mass_kg": mass, "effective_density_kg_m3": mass / item["volume_m3"],
        "com_source_m": item["com_source_m"].tolist(),
    }


def add_mass_custom(model: ET.Element, report: dict) -> None:
    """Store audited masses and COM in native MJCF custom numeric fields."""
    for old in model.findall("custom"):
        model.remove(old)
    selected = report["selected_scenario"]
    center = report["scenarios"][selected]["chassis_com_source_m"]
    whole = report["scenarios"][selected]["assembly_com_source_m"]
    custom = ET.SubElement(model, "custom")
    ET.SubElement(custom, "numeric", name="audit_chassis_mass_kg",
                  data=f"{report['scenarios'][selected]['chassis_mass_kg']:.12g}")
    ET.SubElement(custom, "numeric", name="audit_assembly_mass_kg",
                  data=f"{report['scenarios'][selected]['assembly_mass_kg']:.12g}")
    for name, value in (
        ("audit_populated_pcb_mass_kg", report["scenarios"][selected]["pcb_mass_kg"]),
        ("audit_battery_holder_cells_springs_mass_kg", report["battery_group"]["mass_kg"]),
        ("audit_four_screws_mass_kg", report["screws"]["mass_kg"]),
        ("audit_three_structural_parts_mass_kg", report["structural_parts_group"]["mass_kg"]),
        *((f"audit_{side}_motor_mass_kg", item["mass_kg"])
          for side, item in sorted(report["motor_groups"].items())),
        *((f"audit_{side}_wheel_mass_kg", item["mass_kg"])
          for side, item in sorted(report["wheel_mesh_groups"].items())),
    ):
        ET.SubElement(custom, "numeric", name=name, data=f"{value:.12g}")
    ET.SubElement(custom, "numeric", name="audit_chassis_com_source_m",
                  data=" ".join(f"{v:.12g}" for v in center))
    ET.SubElement(custom, "numeric", name="audit_assembly_com_source_m",
                  data=" ".join(f"{v:.12g}" for v in whole))
    for link_name, record in sorted(report["selected_link_mass_records"].items()):
        ET.SubElement(custom, "numeric", name=f"audit_link_{link_name}",
                      data=" ".join(f"{v:.12g}" for v in
                                    (record["mass_kg"], *record["com_source_m"])))


def add_marker_scene(report: dict) -> None:
    """Make a translucent static COM view; it has no dynamic inertial tags."""
    tree = ET.parse(MODEL_PATH)
    model = tree.getroot()
    model.set("model", "wall_e_com_markers_not_dynamics")
    chassis = model.find("./worldbody/body[@name='chassis_static']")
    if chassis is None:
        raise ValueError("Generated preview has no chassis body")
    # Only the inspection copy changes appearance; the ordinary preview later
    # receives the same numeric metadata without translucent geoms or sites.
    for geom in chassis.findall("geom"):
        rgba = geom.get("rgba", "0.5 0.5 0.5 1").split()
        rgba[3] = "0.28"
        geom.set("rgba", " ".join(rgba))
    selected = report["selected_scenario"]
    center = report["scenarios"][selected]["chassis_com_source_m"]
    whole = report["scenarios"][selected]["assembly_com_source_m"]
    for name, pos, rgba, radius in (
        ("chassis_com_assumption_based", center, "1 0.1 0.12 1", "0.0035"),
        ("assembly_com_assumption_based", whole, "1 0.85 0.05 1", "0.0035"),
        ("battery_group_com", report["battery_group"]["com_source_m"], "0.10 0.85 1 1", "0.0025"),
    ):
        ET.SubElement(chassis, "site", name=name, type="sphere", pos=" ".join(f"{v:.12g}" for v in pos),
                      size=radius, rgba=rgba, group="0")
        ET.SubElement(chassis, "site", name=f"{name}_leader", type="capsule",
                      fromto=" ".join(f"{v:.12g}" for v in (*pos, pos[0], pos[1], 0.082)),
                      size="0.0007", rgba=rgba, group="0")
    add_mass_custom(model, report)
    model.insert(0, ET.Comment("COM sites are inspection markers. Inertials remain omitted; this is not a dynamic model."))
    ET.indent(model, space="  ")
    ET.ElementTree(model).write(MARKERS_PATH, encoding="unicode", xml_declaration=True)


def annotate_preview(report: dict) -> None:
    """Put the same mass ledger metadata in the opaque default preview."""
    tree = ET.parse(MODEL_PATH)
    model = tree.getroot()
    add_mass_custom(model, report)
    ET.indent(model, space="  ")
    tree.write(MODEL_PATH, encoding="unicode", xml_declaration=True)


def build(selected_scenario: str = "pcb_30g") -> dict:
    if selected_scenario not in ("pcb_30g", "pcb_reconstructed"):
        raise ValueError(selected_scenario)
    source = resolve_source(None)
    preview_audit = json.loads(REPORT_PATH.read_text(encoding="utf-8"))
    if preview_audit["source_sha256"] != hashlib.sha256(source.read_bytes()).hexdigest():
        raise ValueError("Preview source hash differs; regenerate the visual preview before auditing mass")
    links, transforms, root, topology, source_mass = load_urdf(source)
    if root != "root":
        raise ValueError("Unexpected source root")
    children = defaultdict(list)
    for joint in ET.parse(source).getroot().findall("joint"):
        children[joint.find("parent").get("link")].append(joint.find("child").get("link"))
    board_names = subtree(BOARD_ROOT, children)
    battery_names = subtree(BATTERY_ROOT, children)
    if not set(SCREWS).issubset(board_names) or len(board_names) != 521 or len(battery_names) != 10:
        raise ValueError("Unexpected board/battery branch structure")
    board_without_screws = [name for name in board_names if name not in SCREWS]
    records = {}
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            records[name] = {"mass_kg": 0.0, "com_source_m": transforms[name][:3, 3].copy()}
            continue
        mass = float(inertial.find("mass").get("value"))
        pose = transforms[name] @ origin_transform(inertial.find("origin"))
        records[name] = {"mass_kg": mass, "com_source_m": pose[:3, 3].copy()}
    np.testing.assert_allclose(sum(record["mass_kg"] for record in records.values()), source_mass, atol=1e-12)
    original_battery_mass = sum(records[name]["mass_kg"] for name in battery_names)
    original_screw_mass = sum(records[name]["mass_kg"] for name in SCREWS)
    original_board_mass = sum(records[name]["mass_kg"] for name in board_without_screws)
    cache = {}
    header = mesh_link_properties("1x8_male_header_001", links, transforms, source, cache)
    header_source_com = records["1x8_male_header_001"]["com_source_m"]
    if header["closed_shells"] != 1 or np.linalg.norm(header["com_source_m"] - header_source_com) > 1e-6:
        raise ValueError("Updated 1x8 male-header mass/COM does not describe its complete mesh")
    battery_items = [mesh_link_properties(name, links, transforms, source, cache) for name in BATTERY_SOLIDS]
    if [item["closed_shells"] for item in battery_items] != [3, 1, 6]:
        raise ValueError("The complete battery group no longer has 3 cells, 1 holder, 6 springs")
    battery_volume = sum(item["volume_m3"] for item in battery_items)
    battery_density = BATTERY_MASS_KG / battery_volume
    battery_masses = [battery_density * item["volume_m3"] for item in battery_items]
    for name in battery_names:
        records[name]["mass_kg"] = 0.0
    for item, mass in zip(battery_items, battery_masses):
        records[item["link"]] = {"mass_kg": mass, "com_source_m": item["com_source_m"]}
    battery_group = aggregate_mesh_group(battery_items, battery_masses)

    screw_items = [mesh_link_properties(name, links, transforms, source, cache) for name in SCREWS]
    if any(item["closed_shells"] != 1 for item in screw_items):
        raise ValueError("A screw mesh is not one closed solid")
    for item in screw_items:
        records[item["link"]] = {"mass_kg": SCREW_MASS_KG, "com_source_m": item["com_source_m"]}
    screw_group = aggregate_mesh_group(screw_items, [SCREW_MASS_KG] * 4)

    # Use the user's individual masses and each transformed mesh centroid.
    structural_items = [mesh_link_properties(name, links, transforms, source, cache)
                        for name in STRUCTURAL_PARTS]
    if any(item["closed_shells"] != 1 for item in structural_items):
        raise ValueError("A structural part is not one closed solid")
    original_structural_parts_mass = sum(records[name]["mass_kg"] for name in STRUCTURAL_PARTS)
    structural_volume = sum(item["volume_m3"] for item in structural_items)
    structural_masses = [STRUCTURAL_PART_MASSES_KG[item["link"]] for item in structural_items]
    for item, mass in zip(structural_items, structural_masses):
        records[item["link"]] = {"mass_kg": mass, "com_source_m": item["com_source_m"]}
    structural_group = {
        **aggregate_mesh_group(structural_items, structural_masses),
        "complete_volume_m3": structural_volume,
        "mass_rule": "User-supplied individual masses: part_1=32 g, part_1_1=4.42 g, part_2=4.42 g.",
        "parts": [json_item(item, mass) for item, mass in zip(structural_items, structural_masses)],
    }

    repeated_overrides, repeated_audit = reconstruct_pcb_repeats(
        links, transforms, source.parent.parent / "meshes")
    if not set(repeated_overrides).issubset(board_without_screws):
        raise ValueError("Repeated electronic links overlap another weighed branch")
    for name, (mass, center) in repeated_overrides.items():
        records[name] = {"mass_kg": mass, "com_source_m": center}

    motor_groups = {}
    for side, names in MOTOR_LINKS.items():
        items = [mesh_link_properties(name, links, transforms, source, cache) for name in names]
        masses = [records[name]["mass_kg"] for name in names]
        if any(item["closed_shells"] != 1 for item in items) or not math.isclose(
                sum(masses), 0.029, rel_tol=0, abs_tol=2e-7):
            raise ValueError(f"Invalid complete {side} motor geometry or mass")
        motor_groups[side] = {
            **aggregate_mesh_group(items, masses),
            "parts": [json_item(item, mass) for item, mass in zip(items, masses)],
            "mass_rule": "CAD per-solid mass retained; three source masses sum to 29 g within export rounding.",
        }
    wheel_mesh_groups = {}
    for key, hinge in topology["hinges"].items():
        names = [name for name, group in topology["groups"].items() if group == key]
        items = [mesh_link_properties(name, links, transforms, source, cache) for name in names]
        masses = [records[name]["mass_kg"] for name in names]
        if len(items) != 2 or any(item["closed_shells"] != 1 for item in items) or not math.isclose(
                sum(masses), 0.040, rel_tol=0, abs_tol=1e-9):
            raise ValueError(f"Invalid complete wheel geometry or mass: {key}")
        wheel_mesh_groups[key] = {
            **aggregate_mesh_group(items, masses),
            "parts": [json_item(item, mass) for item, mass in zip(items, masses)],
            "hinge_source_m": hinge["frame"][:3, 3].tolist(),
            "axis_source": hinge["axis_world"].tolist(),
            "mass_rule": "CAD per-solid rim/tire masses retained; they sum to the measured 40 g wheel.",
        }

    board_reconstructed_mass, board_reconstructed_com = weighted_center(records, board_without_screws)
    if not math.isclose(board_reconstructed_mass - original_board_mass,
                        repeated_audit["added_mass_kg"], rel_tol=0, abs_tol=1e-12):
        raise ValueError("Repeated PCB solids were not assigned exactly once")
    if abs(board_reconstructed_mass - PCB_TARGET_KG) > 0.0005:
        raise ValueError("Reconstructed populated PCB differs from the measured 30 g target by over 0.5 g")
    scenarios = {}
    selected_records = None
    for scenario, board_target in (("pcb_reconstructed", board_reconstructed_mass),
                                   ("pcb_30g", PCB_TARGET_KG)):
        # The tiny final normalization absorbs STL/export decimal rounding;
        # all repeated-instance corrections already have their own true COM.
        factor = board_target / board_reconstructed_mass
        scenario_records = {name: record.copy() for name, record in records.items()}
        for name in board_without_screws:
            scenario_records[name]["mass_kg"] *= factor
        if scenario == selected_scenario:
            selected_records = scenario_records
        chassis_names = [name for name, group in topology["groups"].items() if group == "chassis"]
        chassis_mass, chassis_com = weighted_center(scenario_records, chassis_names)
        assembly_mass, assembly_com = weighted_center(scenario_records, list(links))
        wheel_groups = {}
        for key in sorted(topology["hinges"]):
            names = [name for name, group in topology["groups"].items() if group == key]
            mass, center = weighted_center(scenario_records, names)
            wheel_groups[key] = {"mass_kg": mass, "com_source_m": center.tolist(), "links": names}
            if not math.isclose(mass, 0.040, rel_tol=0, abs_tol=1e-9):
                raise ValueError(f"Wheel mass unexpectedly differs from 40 g: {key}")
        display = np.array(preview_audit["display_translation_m"])
        scenarios[scenario] = {
            "pcb_mass_kg": board_target,
            "pcb_mass_scale_after_repeat_reconstruction": factor,
            "pcb_com_source_m": board_reconstructed_com.tolist(),
            "chassis_mass_kg": chassis_mass,
            "chassis_com_source_m": chassis_com.tolist(),
            "chassis_com_display_m": (chassis_com + display).tolist(),
            "assembly_mass_kg": assembly_mass,
            "assembly_com_source_m": assembly_com.tolist(),
            "assembly_com_display_m": (assembly_com + display).tolist(),
            "wheel_groups": wheel_groups,
            "axle_height_above_source_com_m": float(assembly_com[2] - np.mean(
                [entry["frame"][2, 3] for entry in topology["hinges"].values()])),
        }
    if not math.isclose(scenarios["pcb_30g"]["assembly_mass_kg"] - scenarios["pcb_reconstructed"]["assembly_mass_kg"],
                        PCB_TARGET_KG - board_reconstructed_mass, rel_tol=0, abs_tol=1e-12):
        raise ValueError("PCB mass was double counted")
    report = {
        "mode": "static mass and COM audit; no dynamic chassis inertia, joints, contact, or motors",
        "source_urdf": preview_audit["source_urdf"],
        "source_sha256": preview_audit["source_sha256"],
        "source_exported_mass_kg": source_mass,
        "dependency_versions": preview_audit["dependency_versions"],
        "coordinate_frame": "Source URDF root in metres; display frame adds the preview translation only.",
        "display_translation_m": preview_audit["display_translation_m"],
        "selected_scenario": selected_scenario,
        "selected_link_mass_records": {
            name: {"mass_kg": float(record["mass_kg"]),
                   "com_source_m": record["com_source_m"].tolist(),
                   "rigid_group": topology["groups"][name]}
            for name, record in selected_records.items()
        },
        "updated_male_header": {
            "link": "1x8_male_header_001", "closed_shells": header["closed_shells"],
            "source_mass_kg": records["1x8_male_header_001"]["mass_kg"],
            "mesh_com_source_m": header["com_source_m"].tolist(),
            "source_com_error_m": float(np.linalg.norm(header["com_source_m"] - header_source_com)),
        },
        "user_statements": [
            "All non-battery, non-M3-screw URDF per-instance masses are accepted as correct for this audit; repeated STL solids restore instance multiplicity.",
            "Three cells, holder plastic and six springs together weigh 0.153 kg at one shared effective density.",
            "Each of the four modelled M3 screws weighs 0.0045 kg.",
            "The three structural parts weigh 0.04084 kg combined: part_1 is 0.032 kg and part_1_1 and part_2 are 0.00442 kg each.",
            "The 0.030 kg PCB target refers to the complete populated PCB, including mounted modules and components.",
        ],
        "source_mass_replaced_kg": {"battery_branch": original_battery_mass, "four_screws": original_screw_mass,
                                     "three_structural_parts": original_structural_parts_mass,
                                     "raw_populated_pcb_excluding_screws": original_board_mass},
        "pcb_branch": {"root_link": BOARD_ROOT, "links_excluding_screws": len(board_without_screws),
                       "raw_source_mass_kg": original_board_mass,
                       "reconstructed_mass_kg": board_reconstructed_mass,
                       "reconstructed_com_source_m": board_reconstructed_com.tolist(),
                       "measured_target_mass_kg": PCB_TARGET_KG,
                       "final_normalization_factor": PCB_TARGET_KG / board_reconstructed_mass,
                       "mass_assignment": "For the audited repeated visuals, apply source per-instance mass to every validated equal-volume solid and move its COM to the full visual mesh centroid; zero their placeholders. Then scale the complete populated-board branch to the measured 30 g target, excluding separately weighed screws."},
        "pcb_repeated_solids": repeated_audit,
        "battery_group": {**battery_group, "shared_density_kg_m3": battery_density,
                          "complete_volume_m3": battery_volume,
                          "parts": [json_item(item, mass) for item, mass in zip(battery_items, battery_masses)]},
        "screws": {**screw_group, "parts": [json_item(item, SCREW_MASS_KG) for item in screw_items]},
        "structural_parts_group": structural_group,
        "motor_groups": motor_groups,
        "wheel_mesh_groups": wheel_mesh_groups,
        "scenarios": scenarios,
        "physical_limitations": [
            "Whole-assembly COM depends on the user's assertion that other URDF per-instance masses and inertial-origin COMs are correct; it has not been independently measured.",
            "URDF has appearance colors and source masses, but no named material/density table. CAD per-instance mass and its inferred geometry distribution are accepted as the material weighting for repeated electronics.",
            "Thirty repeated electronics visuals were reconstructed from closed equal-volume solids; other nonmanifold electronics still rely on their original CAD mass and COM.",
            "Full body inertia is not computed: several electronics meshes are nonmanifold and original per-link inertias include invalid tensors.",
            "The corrected assembly changed the 1x8 male header mesh and mass; the reconstructed PCB mass differs from the 30 g target and is scaled uniformly to that target. This is a measured-group assumption, not export rounding.",
            "Watertightness, winding, positive shell volume and units were checked for corrected closed meshes; self-intersections and cross-part overlap were not independently certified.",
        ],
    }
    AUDIT_PATH.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    add_marker_scene(report)
    annotate_preview(report)
    print(f"Wrote {AUDIT_PATH}")
    print(f"Wrote {MARKERS_PATH} (static COM sites, no inertials)")
    for scenario, values in scenarios.items():
        print(f"{scenario}: assembly {values['assembly_mass_kg'] * 1000:.6f} g, "
              f"COM source {np.array(values['assembly_com_source_m']) * 1000} mm")
    return report


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("pcb_30g", "pcb_reconstructed"), default="pcb_30g",
                        help="COM marker choice; both scenarios are always saved in JSON")
    args = parser.parse_args()
    build(args.scenario)
