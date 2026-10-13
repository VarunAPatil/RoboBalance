"""Reconstruct the repeated electronic parts merged into the weighted URDF's STLs.

The export gives one visual link the geometry of several identical instances,
but gives each additional instance an empty 1e-9 kg placeholder link.  The
visual link's mass and inertial origin describe one of those instances.  This
module audits that pattern before replacing each merged visual's mass and COM
with the mass and COM of all of its closed, equal-volume shells.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Mapping
import xml.etree.ElementTree as ET

import numpy as np

from simulation.convert_urdf import numbers, origin_transform, read_stl


@dataclass(frozen=True)
class RepeatFamily:
    prefix: str
    visual_shells: Mapping[str, int]
    placeholders: tuple[str, ...]


# Exact names and shell counts are intentionally pinned to the audited export.
# A changed export must be re-audited rather than silently assigned new masses.
FAMILIES = (
    RepeatFamily(
        "8x1_male_header_pins",
        {name: 2 for name in (
            "8x1_male_header_pins_v2", "8x1_male_header_pins_v2_1",
            "8x1_male_header_pins_v2_1_1", "8x1_male_header_pins_v2_1_2",
            "8x1_male_header_pins_v2_1_3", "8x1_male_header_pins_v2_1_4",
            "8x1_male_header_pins_v2_1_5", "8x1_male_header_pins_v2_1_6",
            "8x1_male_header_pins_v2_2",
        )},
        (
            "8x1_male_header_pins_v2_1__3_", "8x1_male_header_pins_v2_1__4_",
            "8x1_male_header_pins_v2__1_", "8x1_male_header_pins_v2__2_",
            "8x1_male_header_pins_v2__5_", "8x1_male_header_pins_v2__6_",
            "8x1_male_header_pins_v2__7_", "8x1_male_header_pins_v2__8_",
            "8x1_male_header_pins_v2__9_",
        ),
    ),
    RepeatFamily(
        "c_0805_2012metric",
        {"c_0805_2012metric_10": 12},
        tuple(["c_0805_2012metric__1_"] +
              [f"c_0805_2012metric_{i}__1_" for i in (*range(1, 10), 11)]),
    ),
    RepeatFamily(
        "c_1206_3216metric",
        {"c_1206_3216metric_2": 6},
        tuple(["c_1206_3216metric__1_"] +
              [f"c_1206_3216metric_{i}__1_" for i in (1, 3, 4, 5)]),
    ),
    RepeatFamily("d_smb", {"d_smb": 2}, ("d_smb_1__1_",)),
    RepeatFamily(
        "jst_b2b_eh_a", {"jst_b2b_eh_a_2": 3},
        ("jst_b2b_eh_a__1_", "jst_b2b_eh_a_1__1_"),
    ),
    RepeatFamily(
        "led_0805_2012metric", {"led_0805_2012metric_51": 67},
        tuple(["led_0805_2012metric__1_"] +
              [f"led_0805_2012metric_{i}__1_" for i in range(1, 67) if i != 51]),
    ),
    RepeatFamily(
        "msk12c02",
        {name: 2 for name in (
            "msk12c02", "msk12c02_1", "msk12c02_1_1", "msk12c02_1_2",
            "msk12c02_1_3", "msk12c02_1_4", "msk12c02_2", "msk12c02_3",
        )},
        (
            "msk12c02_1__3_", "msk12c02_1__4_", "msk12c02_1__5_",
            "msk12c02__1_", "msk12c02__2_", "msk12c02__6_",
            "msk12c02__7_", "msk12c02__8_",
        ),
    ),
    RepeatFamily(
        "pinheader_",
        {
            "pinheader_1x03_p254mm_vertical": 7,
            "pinheader_1x10_p254mm_vertical": 1,
            "pinheader_1x12_p254mm_vertical_1": 2,
        },
        tuple([f"pinheader_1x03_p254mm_vertical_{i}__1_" for i in range(1, 7)] +
              ["pinheader_1x12_p254mm_vertical__1_"]),
    ),
    RepeatFamily(
        "pinsocket_",
        {"pinsocket_1x04_p2_54mm_vertical": 1,
         "pinsocket_1x08_p2_54mm_vertical_1": 3},
        ("pinsocket_1x08_p2_54mm_vertical__1_",
         "pinsocket_1x08_p2_54mm_vertical_2__1_"),
    ),
    RepeatFamily(
        "r_0805_2012metric", {"r_0805_2012metric_19": 21},
        tuple(["r_0805_2012metric__1_"] +
              [f"r_0805_2012metric_{i}__1_" for i in range(1, 21) if i != 19]),
    ),
    RepeatFamily("sot_223", {"sot_223": 2}, ("sot_223_1__1_",)),
    RepeatFamily("sw_spst_pts810", {"sw_spst_pts810": 2},
                 ("sw_spst_pts810_1__1_",)),
)


def _mass_and_com(link: ET.Element, link_transform: np.ndarray) -> tuple[float, np.ndarray]:
    inertial = link.find("inertial")
    if inertial is None:
        raise ValueError(f"Missing inertial on {link.get('name')}")
    mass = numbers(inertial.find("mass"), "value", (0,), 1)[0]
    if mass <= 0:
        raise ValueError(f"Non-positive source mass on {link.get('name')}")
    com = (link_transform @ origin_transform(inertial.find("origin")))[:3, 3]
    return float(mass), com


def _mesh_shells(
    link_name: str, link: ET.Element, link_transform: np.ndarray, mesh_root: Path,
) -> tuple[np.ndarray, np.ndarray, str, str]:
    visuals = link.findall("visual")
    if len(visuals) != 1:
        raise ValueError(f"Expected one visual for {link_name}, got {len(visuals)}")
    visual = visuals[0]
    mesh = visual.find("./geometry/mesh")
    if mesh is None:
        raise ValueError(f"Expected mesh visual for {link_name}")
    prefix = "package://assembly_1/meshes/"
    uri = mesh.get("filename", "")
    if not uri.startswith(prefix):
        raise ValueError(f"Unexpected mesh URI for {link_name}: {uri}")
    path = (mesh_root / uri[len(prefix):]).resolve()
    try:
        path.relative_to(mesh_root)
    except ValueError as exc:
        raise ValueError(f"Mesh escapes package for {link_name}: {uri}") from exc
    if not path.is_file():
        raise FileNotFoundError(path)
    scale = numbers(mesh, "scale", (1, 1, 1), 3)
    if np.any(scale <= 0):
        raise ValueError(f"Non-positive scale for {link_name}")
    vertices = read_stl(path).astype(float)
    unique_vertices, inverse = np.unique(vertices, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    if np.any(faces[:, 0] == faces[:, 1]) or np.any(faces[:, 1] == faces[:, 2]) or np.any(faces[:, 2] == faces[:, 0]):
        raise ValueError(f"Degenerate face in {path.name}")
    edges = np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    _, edge_ids = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True)
    incidence = np.bincount(edge_ids)
    orientation = np.bincount(edge_ids, weights=np.where(edges[:, 0] < edges[:, 1], 1, -1))
    if np.any(incidence != 2) or np.any(orientation != 0):
        raise ValueError(f"Mesh is not closed and consistently wound: {path.name}")

    parent = np.arange(len(unique_vertices))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    for a, b, c in faces:
        root = find(int(a))
        parent[find(int(b))] = root
        parent[find(int(c))] = root
    face_roots = np.fromiter((find(int(i)) for i in faces[:, 0]), dtype=np.int64, count=len(faces))
    _, component_ids = np.unique(face_roots, return_inverse=True)
    triangles = (vertices.reshape(-1, 3, 3) * scale)
    areas = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                    triangles[:, 2] - triangles[:, 0]), axis=1)
    if np.any(areas <= 0):
        raise ValueError(f"Zero-area face in {path.name}")
    signed_tetra = np.einsum("ij,ij->i", triangles[:, 0],
                             np.cross(triangles[:, 1], triangles[:, 2])) / 6
    volumes = np.bincount(component_ids, weights=signed_tetra)
    if np.any(volumes <= 0) or not np.isfinite(volumes).all():
        raise ValueError(f"Non-positive shell volume in {path.name}")
    face_moments = signed_tetra[:, None] * triangles.sum(axis=1) / 4
    moments = np.column_stack([
        np.bincount(component_ids, weights=face_moments[:, axis]) for axis in range(3)
    ])
    local_coms = moments / volumes[:, None]
    world = link_transform @ origin_transform(visual.find("origin"))
    world_coms = local_coms @ world[:3, :3].T + world[:3, 3]
    return volumes, world_coms, path.name, hashlib.sha256(path.read_bytes()).hexdigest()


def reconstruct_pcb_repeats(
    links: Mapping[str, ET.Element], transforms: Mapping[str, np.ndarray], mesh_root: Path,
) -> tuple[dict[str, tuple[float, np.ndarray]], dict]:
    """Return corrected (kg, source-frame COM in m) records for PCB repeats.

    Overrides include all 30 visual links and all 133 empty placeholders.  A
    placeholder's corrected mass is zero; its COM is retained only for a
    well-formed record.  The caller must apply these overrides once, alongside
    separate battery and screw corrections, without adding a second 30 g PCB.
    """
    mesh_root = Path(mesh_root).resolve()
    overrides: dict[str, tuple[float, np.ndarray]] = {}
    families = []
    max_volume_ratio = 1.0
    max_single_com_error = 0.0
    for family in FAMILIES:
        expected = set(family.visual_shells) | set(family.placeholders)
        actual = {name for name in links if name.startswith(family.prefix)}
        if actual != expected:
            raise ValueError(f"Repeat family {family.prefix} changed: missing={sorted(expected - actual)}, extra={sorted(actual - expected)}")
        if sum(family.visual_shells.values()) != len(expected) or len(family.visual_shells) + len(family.placeholders) != len(expected):
            raise AssertionError(f"Incorrect expected shell/placeholder mapping: {family.prefix}")
        source_mass = 0.0
        corrected_mass = 0.0
        visuals = []
        for name, expected_shells in family.visual_shells.items():
            if name in overrides:
                raise ValueError(f"Repeated link assigned twice: {name}")
            link = links[name]
            mass, original_com = _mass_and_com(link, transforms[name])
            volumes, world_coms, filename, mesh_sha256 = _mesh_shells(
                name, link, transforms[name], mesh_root)
            if len(volumes) != expected_shells:
                raise ValueError(f"Expected {expected_shells} closed shells for {name}, found {len(volumes)}")
            ratio = float(volumes.max() / volumes.min())
            if ratio > 1.0001:
                raise ValueError(f"Copied shells have unequal volumes in {filename}: ratio={ratio}")
            max_volume_ratio = max(max_volume_ratio, ratio)
            nearest_error = float(np.linalg.norm(world_coms - original_com, axis=1).min())
            if nearest_error > 1e-6:
                raise ValueError(f"URDF inertial COM does not match one shell in {filename}: {nearest_error} m")
            max_single_com_error = max(max_single_com_error, nearest_error)
            full_com = np.average(world_coms, axis=0, weights=volumes)
            new_mass = expected_shells * mass
            overrides[name] = (new_mass, full_com if expected_shells > 1 else original_com)
            source_mass += mass
            corrected_mass += new_mass
            visuals.append({
                "link": name, "mesh": filename, "mesh_sha256": mesh_sha256,
                "shells": expected_shells,
                "source_mass_kg": mass, "corrected_mass_kg": new_mass,
                "shell_volume_ratio": ratio,
                "source_com_to_nearest_shell_m": nearest_error,
            })
        for name in family.placeholders:
            if name in overrides:
                raise ValueError(f"Repeated link assigned twice: {name}")
            link = links[name]
            if link.findall("visual"):
                raise ValueError(f"Placeholder unexpectedly has a visual: {name}")
            mass, com = _mass_and_com(link, transforms[name])
            if not np.isclose(mass, 1e-9, atol=1e-15, rtol=0):
                raise ValueError(f"Placeholder mass changed: {name}={mass}")
            source_mass += mass
            overrides[name] = (0.0, com)
        families.append({
            "prefix": family.prefix, "visuals": visuals,
            "placeholders": list(family.placeholders),
            "source_mass_kg": source_mass, "corrected_mass_kg": corrected_mass,
            "added_mass_kg": corrected_mass - source_mass,
        })
    if len(overrides) != 163:
        raise AssertionError(f"Expected 30 visual and 133 placeholder overrides, got {len(overrides)}")
    source_total = sum(f["source_mass_kg"] for f in families)
    corrected_total = sum(f["corrected_mass_kg"] for f in families)
    audit = {
        "family_count": len(families), "visual_link_count": 30, "placeholder_link_count": 133,
        "source_mass_kg": source_total, "corrected_mass_kg": corrected_total,
        "added_mass_kg": corrected_total - source_total,
        "max_shell_volume_ratio": max_volume_ratio,
        "max_original_com_to_nearest_shell_m": max_single_com_error,
        "families": families,
    }
    return overrides, audit
