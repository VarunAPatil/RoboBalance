"""Build a provisional, screenshot-derived Wall-E dynamics inspection model.

The connector pose is inferred from three corresponding COM observations.  It is
not a direct CAD connector export, so the builder rejects changed source/audit
data or a poor rigid fit instead of guessing a frame.
"""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from simulation.convert_urdf import load_urdf, quat_wxyz, resolve_source


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "simulation/models"
PREVIEW = MODEL_DIR / "wall_e_preview.xml"
PREVIEW_AUDIT = MODEL_DIR / "wall_e_preview_audit.json"
MASS_AUDIT = MODEL_DIR / "wall_e_mass_audit.json"
OUTPUT = MODEL_DIR / "wall_e_dynamic.xml"
AUDIT = MODEL_DIR / "wall_e_dynamic_audit.json"
SCENARIO = "pcb_reconstructed"

# Exactly the numbers displayed in the three screenshots.  COM is mm and the
# symmetric tensors are g*mm^2, all about Mate connector 1 of Rim <2>.
SCREENSHOT = {
    "full": {
        "mass_g": 379.768,
        "com_connector_mm": [19.05, -8.18, -91.98],
        "inertia_connector_g_mm2": [[4275000, 67658.407, 664548.976],
                                     [67658.407, 4376000, -284794.478],
                                     [664548.976, -284794.478, 497115.056]],
    },
    "left": {
        "mass_g": 40.0,
        "com_connector_mm": [2.93e-5, 0.0, -176.63],
        "inertia_connector_g_mm2": [[1264000, -0.405, 0.202],
                                     [-0.405, 1264000, 0.063],
                                     [0.202, 0.063, 27124.084]],
    },
    "right": {
        "mass_g": 40.0,
        "com_connector_mm": [0.0, 2.94e-5, -7.37],
        "inertia_connector_g_mm2": [[18312.789, 0.001, -0.005],
                                     [0.001, 18311.904, 0.013],
                                     [-0.005, 0.013, 27124.084]],
    },
}


def fmt(values: np.ndarray | list[float]) -> str:
    return " ".join(f"{float(value):.16g}" for value in values)


def parallel_axis(mass: float, displacement: np.ndarray) -> np.ndarray:
    return mass * (np.dot(displacement, displacement) * np.eye(3)
                   - np.outer(displacement, displacement))


def inertia_checks(label: str, tensor: np.ndarray) -> list[float]:
    if not np.isfinite(tensor).all() or not np.allclose(tensor, tensor.T, atol=1e-13):
        raise ValueError(f"{label}: nonfinite or nonsymmetric inertia")
    principal = np.linalg.eigvalsh(tensor)
    if principal[0] <= 0 or principal[-1] > principal[0] + principal[1]:
        raise ValueError(f"{label}: nonphysical principal inertias {principal}")
    return principal.tolist()


def infer_connector_pose(scenario: dict) -> tuple[np.ndarray, np.ndarray, dict]:
    # Three non-collinear mass-centre correspondences determine all six pose
    # coordinates.  A proper Kabsch rotation is fitted because displayed COMs
    # are rounded to ~0.01 mm.  No screenshot camera-axis mapping is assumed.
    source_mm = np.array([
        scenario["wheel_groups"]["revolute_2"]["com_source_m"],
        scenario["wheel_groups"]["revolute_1"]["com_source_m"],
        scenario["assembly_com_source_m"],
    ]) * 1000
    connector_mm = np.array([
        SCREENSHOT["right"]["com_connector_mm"],
        SCREENSHOT["left"]["com_connector_mm"],
        SCREENSHOT["full"]["com_connector_mm"],
    ])
    x = connector_mm - connector_mm.mean(axis=0)
    y = source_mm - source_mm.mean(axis=0)
    u, _, vt = np.linalg.svd(x.T @ y)
    adjust = np.diag([1, 1, np.linalg.det(u @ vt)])
    row_rotation = u @ adjust @ vt  # connector row vectors -> source rows
    rotation = row_rotation.T  # connector column vectors -> source columns
    origin_mm = source_mm.mean(axis=0) - rotation @ connector_mm.mean(axis=0)
    predictions = connector_mm @ rotation.T + origin_mm
    residual = predictions - source_mm
    pairwise_errors = [abs(np.linalg.norm(source_mm[i] - source_mm[j])
                           - np.linalg.norm(connector_mm[i] - connector_mm[j]))
                       for i, j in ((0, 1), (0, 2), (1, 2))]
    if np.linalg.det(rotation) < 0.999999 or max(pairwise_errors) > 0.02 or np.max(np.abs(residual)) > 0.01:
        raise ValueError("Screenshot and audit COM points do not define a defensible rigid frame")
    fit = {
        "method": "proper rigid Kabsch fit: right wheel COM, left wheel COM, full assembly COM",
        "source_correspondences_mm": source_mm.tolist(),
        "screenshot_correspondences_connector_mm": connector_mm.tolist(),
        "pairwise_distance_errors_mm": pairwise_errors,
        "fit_residual_source_mm": residual.tolist(),
        "maximum_fit_residual_mm": float(np.max(np.abs(residual))),
        "connector_origin_source_m": (origin_mm / 1000).tolist(),
        "rotation_connector_to_source": rotation.tolist(),
        "frame_status": "Inferred from rounded COMs, not an explicit mate-connector CAD pose export.",
    }
    return rotation, origin_mm / 1000, fit


def fullinertia(tensor: np.ndarray) -> str:
    return fmt([tensor[0, 0], tensor[1, 1], tensor[2, 2],
                tensor[0, 1], tensor[0, 2], tensor[1, 2]])


def build() -> dict:
    source = resolve_source(None)
    mass_audit = json.loads(MASS_AUDIT.read_text())
    preview_audit = json.loads(PREVIEW_AUDIT.read_text())
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != mass_audit["source_sha256"] or digest != preview_audit["source_sha256"]:
        raise ValueError("Source and mass/visual audits differ; rebuild audits first")
    if mass_audit["source_urdf"] != preview_audit["source_urdf"]:
        raise ValueError("Mass and visual audits use different URDFs")
    scenario = mass_audit["scenarios"][SCENARIO]
    if abs(scenario["assembly_mass_kg"] * 1000 - SCREENSHOT["full"]["mass_g"]) > 0.001:
        raise ValueError("Selected ledger no longer matches screenshot total to 1 mg")
    structural = mass_audit["structural_parts_group"]["parts"]
    expected = {"part_1": 0.032, "part_1_1": 0.00442, "part_2": 0.00442}
    if {part["link"]: part["mass_kg"] for part in structural} != expected:
        raise ValueError("Structural mass assignments changed")
    _, _, _, topology, _ = load_urdf(source)
    rotation, connector_origin_source, fit = infer_connector_pose(scenario)

    masses = {key: screenshot["mass_g"] * 1e-3 for key, screenshot in SCREENSHOT.items()}
    com_connector = {key: np.array(item["com_connector_mm"]) * 1e-3
                     for key, item in SCREENSHOT.items()}
    inertia_connector = {key: np.array(item["inertia_connector_g_mm2"]) * 1e-9
                         for key, item in SCREENSHOT.items()}
    # Shift screenshot tensors with screenshot masses only.  The 0.000177 g
    # ledger difference is retained explicitly in MJCF mass and reported; no
    # tensor scaling or hidden mass correction is applied.
    masses["chassis"] = masses["full"] - masses["left"] - masses["right"]
    if abs(scenario["chassis_mass_kg"] - masses["chassis"]) > 2e-7:
        raise ValueError("Screenshot and ledger masses no longer reconcile")
    com_connector["chassis"] = (
        masses["full"] * com_connector["full"]
        - masses["left"] * com_connector["left"]
        - masses["right"] * com_connector["right"]
    ) / (masses["full"] - masses["left"] - masses["right"])
    inertia_connector["chassis"] = (inertia_connector["full"]
                                     - inertia_connector["left"]
                                     - inertia_connector["right"])
    body = {}
    for label, hinge_key in (("chassis", None), ("left", "revolute_1"), ("right", "revolute_2")):
        mass = masses[label]
        model_mass = scenario["chassis_mass_kg"] if label == "chassis" else mass
        connector_com = com_connector[label]
        # The chassis connector-origin tensor was formed by direct subtraction;
        # wheel connector-origin tensors were read from their individual images.
        shifted = inertia_connector[label] - parallel_axis(mass, connector_com)
        principal = inertia_checks(label, shifted)
        source_inertia = rotation @ shifted @ rotation.T
        screenshot_com_source = connector_origin_source + rotation @ connector_com
        if hinge_key is None:
            body_rotation = np.eye(3)
            body_origin = np.zeros(3)
            ledger_com = np.array(scenario["chassis_com_source_m"])
        else:
            frame = topology["hinges"][hinge_key]["frame"]
            body_rotation, body_origin = frame[:3, :3], frame[:3, 3]
            ledger_com = np.array(scenario["wheel_groups"][hinge_key]["com_source_m"])
        error_mm = (screenshot_com_source - ledger_com) * 1000
        if np.linalg.norm(error_mm) > 0.03:
            raise ValueError(f"{label}: screenshot and ledger COM mismatch {error_mm} mm")
        local_com = body_rotation.T @ (ledger_com - body_origin)
        local_inertia = body_rotation.T @ source_inertia @ body_rotation
        inertia_checks(f"{label} local", local_inertia)
        _, principal_axes_body = np.linalg.eigh(local_inertia)
        if np.linalg.det(principal_axes_body) < 0:
            principal_axes_body[:, 0] *= -1
        body[label] = {
            "mass_kg": model_mass,
            "screenshot_mass_kg_used_for_parallel_axis": mass,
            "com_connector_m_from_screenshot": connector_com.tolist(),
            "com_source_m_from_screenshot_mapping": screenshot_com_source.tolist(),
            "com_source_m_used_from_ledger": ledger_com.tolist(),
            "screenshot_to_ledger_com_error_mm": error_mm.tolist(),
            "com_body_m": local_com.tolist(),
            "inertia_connector_about_connector_kg_m2": inertia_connector[label].tolist(),
            "parallel_axis_subtracted_kg_m2": parallel_axis(mass, connector_com).tolist(),
            "inertia_connector_about_com_kg_m2": shifted.tolist(),
            "inertia_source_about_com_kg_m2": source_inertia.tolist(),
            "rotation_body_to_source": body_rotation.tolist(),
            "inertia_body_about_com_kg_m2": local_inertia.tolist(),
            "principal_inertias_kg_m2": principal,
            "inertial_frame_quat_wxyz": [1.0, 0.0, 0.0, 0.0],
            "inertial_representation": "MuJoCo fullinertia in each body frame; no principal-axis rotation used",
            "principal_axes_to_body_rotation": principal_axes_body.tolist(),
            "principal_axes_to_body_quat_wxyz_numerical": quat_wxyz(principal_axes_body).tolist(),
        }

    tree = ET.parse(PREVIEW)
    model = tree.getroot()
    model.set("model", "wall_e_cad_screenshot_provisional_dynamics")
    model.find("option").set("gravity", "0 0 -9.81")
    # The source preview carries pcb_30g-only custom numeric annotations.
    # They are not physical properties and would conflict with this model's
    # pcb_reconstructed body masses, so keep its JSON audit separately.
    custom = model.find("custom")
    if custom is not None:
        model.remove(custom)
    world = model.find("worldbody")
    floor = world.find("geom[@name='visual_floor']")
    floor.set("name", "floor_collision_and_visual")
    floor.set("contype", "1")
    floor.set("conaffinity", "1")
    chassis = world.find("body[@name='chassis_static']")
    chassis.set("name", "chassis")
    # Existing visual transform lifts all geometry 3 mm.  Move down 3.01 mm:
    # compiled tire-floor gaps are about 0.499 mm, so this leaves only ~0.01 mm
    # tire penetration at qpos=0 and ensures contact is active on the first step.
    initial_translation = np.array(preview_audit["display_translation_m"])
    initial_translation[2] -= 0.00301
    chassis.set("pos", fmt(initial_translation))
    ET.SubElement(chassis, "freejoint", name="chassis_free")
    ET.SubElement(chassis, "inertial", mass=f"{body['chassis']['mass_kg']:.16g}",
                  pos=fmt(body["chassis"]["com_body_m"]),
                  fullinertia=fullinertia(np.array(body["chassis"]["inertia_body_about_com_kg_m2"])))
    wheel_names = {}
    for label, hinge_key in (("left", "revolute_1"), ("right", "revolute_2")):
        wheel = chassis.find(f"body[@name='wheel_{label}_static']")
        wheel.set("name", f"wheel_{label}")
        axis = topology["hinges"][hinge_key]["axis_local"]
        ET.SubElement(wheel, "joint", name=f"wheel_{label}_hinge", type="hinge",
                      pos="0 0 0", axis=fmt(axis), damping="0")
        ET.SubElement(wheel, "inertial", mass=f"{body[label]['mass_kg']:.16g}",
                      pos=fmt(body[label]["com_body_m"]),
                      fullinertia=fullinertia(np.array(body[label]["inertia_body_about_com_kg_m2"])))
        # Tire STL is approximately 68.9 mm in outside diameter and 24.8 mm
        # across the axle.  This simple cylinder is a provisional proxy only.
        tire_link = "tyre" if label == "left" else "tyre_1"
        tire = next(item for item in mass_audit["wheel_mesh_groups"][hinge_key]["parts"]
                    if item["link"] == tire_link)
        frame = topology["hinges"][hinge_key]["frame"]
        proxy_center = frame[:3, :3].T @ (np.array(tire["com_source_m"]) - frame[:3, 3])
        ET.SubElement(wheel, "geom", name=f"wheel_{label}_collision", type="cylinder",
                      pos=fmt(proxy_center), size="0.03446 0.01245", contype="1",
                      conaffinity="1", rgba="0 0 0 0", friction="1 0.005 0.0001")
        wheel_names[label] = hinge_key
    model.insert(0, ET.Comment("Provisional CAD/screenshot dynamics. Connector pose inferred from rounded COMs; no physical validation."))
    model.insert(1, ET.Comment("Only floor and simple tire cylinders collide. Friction uses provisional MuJoCo defaults; no motors."))
    # Preserve all 404 detailed mesh geoms and their original colors/poses.
    if len(model.findall(".//geom[@type='mesh']")) != preview_audit["visual_instances"]:
        raise ValueError("Detailed visual geometry changed")
    ET.indent(model, space="  ")
    tree.write(OUTPUT, encoding="unicode", xml_declaration=True)
    report = {
        "status": "CAD/screenshot-derived provisional; not hardware-validated",
        "source_urdf": mass_audit["source_urdf"], "source_sha256": digest,
        "selected_mass_scenario": SCENARIO,
        "scenario_assembly_mass_kg": scenario["assembly_mass_kg"],
        "screenshot_full_mass_kg": masses["full"],
        "mass_difference_screenshot_minus_ledger_kg": masses["full"] - scenario["assembly_mass_kg"],
        "alternative_pcb_30g_assembly_mass_kg": mass_audit["scenarios"]["pcb_30g"]["assembly_mass_kg"],
        "structural_part_masses_kg": expected,
        "screenshot_values_and_units": SCREENSHOT,
        "connector_pose_in_source": fit,
        "body_properties": body,
        "hinge_urdf_frames": {
            label: {"joint": key, "origin_source_m": topology["hinges"][key]["frame"][:3, 3].tolist(),
                    "rotation_body_to_source": topology["hinges"][key]["frame"][:3, :3].tolist(),
                    "axis_local": topology["hinges"][key]["axis_local"].tolist(),
                    "axis_source": topology["hinges"][key]["axis_world"].tolist()}
            for label, key in wheel_names.items()},
        "initial_chassis_translation_source_to_world_m": initial_translation.tolist(),
        "collision": "Floor plane and two transparent cylinder tire proxies (radius 0.03446 m, half-width 0.01245 m); friction 1, 0.005, 0.0001 is provisional, not measured.",
        "precision_limits": [
            "Full Lxx/Lyy and left-wheel Lxx/Lyy are shown with four significant digits; chassis subtraction inherits their rounding uncertainty.",
            "Screenshot COMs are rounded to 0.01 mm; the connector pose is fitted from three COM correspondences, not read directly from CAD.",
            "The selected mass ledger differs from screenshot mass by 0.000177 g; no tensor scaling was applied.",
            "Inertial positions use audited source-frame COMs; mapped screenshot COMs agree within the recorded residuals.",
            "No chassis collision shape, calibrated contact, motor, sensor, or physical mass/inertia validation is provided.",
        ],
    }
    AUDIT.write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    result = build()
    print(f"Wrote {OUTPUT} and {AUDIT}")
    print(f"Connector fit maximum residual: {result['connector_pose_in_source']['maximum_fit_residual_mm']:.6f} mm")
