"""Verify the corrected mass/COM ledger and its static inspection scene."""

from __future__ import annotations

import hashlib
import json
import math

import mujoco
import numpy as np

from simulation.convert_urdf import DEFAULT_SOURCE, MODEL_PATH, PROJECT, REPORT_PATH
from simulation.mass_audit import AUDIT_PATH, MARKERS_PATH


def main() -> None:
    report = json.loads(AUDIT_PATH.read_text(encoding="utf-8"))
    visual_report = json.loads(REPORT_PATH.read_text(encoding="utf-8"))
    source = PROJECT / report["source_urdf"]
    assert source.resolve() == DEFAULT_SOURCE.resolve(), "Mass audit uses the wrong assembly"
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    assert source_hash == report["source_sha256"] == visual_report["source_sha256"]
    assert report["selected_scenario"] in report["scenarios"]
    selected = report["scenarios"][report["selected_scenario"]]
    assert math.isclose(report["battery_group"]["mass_kg"], 0.153, abs_tol=1e-12)
    assert [part["closed_shells"] for part in report["battery_group"]["parts"]] == [3, 1, 6]
    assert math.isclose(sum(part["mass_kg"] for part in report["battery_group"]["parts"]),
                        0.153, abs_tol=1e-12)
    assert len(report["screws"]["parts"]) == 4
    assert all(math.isclose(part["mass_kg"], 0.0045, abs_tol=1e-12) for part in report["screws"]["parts"])
    assert math.isclose(report["screws"]["mass_kg"], 0.018, abs_tol=1e-12)
    structural = report["structural_parts_group"]
    assert [part["link"] for part in structural["parts"]] == ["part_1", "part_1_1", "part_2"]
    expected_structural_masses = {"part_1": 0.032, "part_1_1": 0.00442, "part_2": 0.00442}
    assert math.isclose(structural["mass_kg"], 0.04084, abs_tol=1e-12)
    assert {part["link"]: part["mass_kg"] for part in structural["parts"]} == expected_structural_masses
    assert math.isclose(selected["pcb_mass_kg"], 0.030, abs_tol=1e-12)
    repeats = report["pcb_repeated_solids"]
    assert repeats["family_count"] == 12 and repeats["visual_link_count"] == 30
    assert repeats["placeholder_link_count"] == 133
    assert math.isclose(repeats["added_mass_kg"], 0.003566976549, abs_tol=1e-12)
    board = report["pcb_branch"]
    assert math.isclose(board["reconstructed_mass_kg"],
                        board["raw_source_mass_kg"] + repeats["added_mass_kg"], abs_tol=1e-12)
    assert abs(board["reconstructed_mass_kg"] - 0.030) < 0.0005
    assert board["links_excluding_screws"] == 517
    replaced = report["source_mass_replaced_kg"]
    expected_total = (report["source_exported_mass_kg"] - replaced["battery_branch"]
                      - replaced["four_screws"] + 0.153 + 4 * 0.0045
                      - replaced["three_structural_parts"] + sum(expected_structural_masses.values())
                      + repeats["added_mass_kg"] + (0.030 - board["reconstructed_mass_kg"]))
    assert math.isclose(selected["assembly_mass_kg"], expected_total, abs_tol=1e-12)
    assert math.isclose(selected["assembly_mass_kg"], selected["chassis_mass_kg"] + 0.080,
                        abs_tol=1e-12)
    records = report["selected_link_mass_records"]
    assert len(records) == visual_report["source_links"] == 545
    assert math.isclose(sum(record["mass_kg"] for record in records.values()),
                        selected["assembly_mass_kg"], abs_tol=1e-12)
    for group, expected in (("chassis", selected["chassis_mass_kg"]),
                            *((name, values["mass_kg"]) for name, values in selected["wheel_groups"].items())):
        assert math.isclose(sum(record["mass_kg"] for record in records.values()
                                if record["rigid_group"] == group), expected, abs_tol=1e-12)
    assert report["updated_male_header"]["closed_shells"] == 1
    assert report["updated_male_header"]["source_com_error_m"] < 1e-6
    for group in report["motor_groups"].values():
        assert math.isclose(group["mass_kg"], 0.029, abs_tol=2e-7)
    for group in report["wheel_mesh_groups"].values():
        assert math.isclose(group["mass_kg"], 0.040, abs_tol=1e-9)
    for family in ("battery_group", "screws"):
        for part in report[family]["parts"]:
            path = source.parent.parent / "meshes" / part["mesh"]
            assert hashlib.sha256(path.read_bytes()).hexdigest() == part["mesh_sha256"]
    for family in repeats["families"]:
        for visual in family["visuals"]:
            path = source.parent.parent / "meshes" / visual["mesh"]
            assert hashlib.sha256(path.read_bytes()).hexdigest() == visual["mesh_sha256"]

    model = mujoco.MjModel.from_xml_path(str(MARKERS_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.nbody == 4 and model.njnt == 0 and model.ngeom == visual_report["visual_instances"] + 1 and model.nsite == 6
    assert model.nmesh == visual_report["unique_mesh_assets"]
    assert np.all(model.body_mass == 0) and np.all(model.body_inertia == 0)
    assert np.all(model.geom_contype == 0) and np.all(model.geom_conaffinity == 0)
    mass_metadata = (("audit_chassis_mass_kg", selected["chassis_mass_kg"]),
                           ("audit_assembly_mass_kg", selected["assembly_mass_kg"]),
                           ("audit_populated_pcb_mass_kg", selected["pcb_mass_kg"]),
                           ("audit_battery_holder_cells_springs_mass_kg", report["battery_group"]["mass_kg"]),
                           ("audit_four_screws_mass_kg", report["screws"]["mass_kg"]),
                           ("audit_three_structural_parts_mass_kg", structural["mass_kg"]),
                           *((f"audit_{side}_motor_mass_kg", values["mass_kg"])
                             for side, values in sorted(report["motor_groups"].items())),
                           *((f"audit_{side}_wheel_mass_kg", values["mass_kg"])
                             for side, values in sorted(report["wheel_mesh_groups"].items())))
    for scene in (model, mujoco.MjModel.from_xml_path(str(MODEL_PATH))):
        assert np.all(scene.body_mass == 0) and np.all(scene.body_inertia == 0)
        for name, expected in mass_metadata:
            numeric_id = mujoco.mj_name2id(scene, mujoco.mjtObj.mjOBJ_NUMERIC, name)
            assert numeric_id >= 0 and scene.numeric_size[numeric_id] == 1
            assert math.isclose(scene.numeric_data[scene.numeric_adr[numeric_id]], expected, abs_tol=1e-10)
        for link_name, record in records.items():
            numeric_id = mujoco.mj_name2id(scene, mujoco.mjtObj.mjOBJ_NUMERIC, f"audit_link_{link_name}")
            assert numeric_id >= 0 and scene.numeric_size[numeric_id] == 4
            np.testing.assert_allclose(
                scene.numeric_data[scene.numeric_adr[numeric_id]:scene.numeric_adr[numeric_id] + 4],
                [record["mass_kg"], *record["com_source_m"]], atol=1e-10, rtol=0)
    for site_name, key in (
        ("chassis_com_assumption_based", "chassis_com_display_m"),
        ("assembly_com_assumption_based", "assembly_com_display_m"),
    ):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        assert site_id >= 0
        np.testing.assert_allclose(data.site_xpos[site_id], selected[key], atol=1e-9, rtol=0)
    battery_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "battery_group_com")
    assert battery_site >= 0
    np.testing.assert_allclose(data.site_xpos[battery_site],
                               np.array(report["battery_group"]["com_source_m"]) +
                               np.array(report["display_translation_m"]), atol=1e-9, rtol=0)
    print(f"Verified selected {report['selected_scenario']}: {selected['assembly_mass_kg'] * 1000:.6f} g")
    print(f"Assembly COM in source frame (mm): {np.array(selected['assembly_com_source_m']) * 1000}")
    print("Static MuJoCo COM sites and mass metadata compile; no physical inertials or motion were added")


if __name__ == "__main__":
    main()
