"""Independent geometry checks for the Wall-E mass and COM audit."""

from __future__ import annotations

import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from simulation.mass_audit import aggregate_mesh_group, solid_properties


PROJECT = Path(__file__).resolve().parents[1]


def cube_triangles(origin=(0.0, 0.0, 0.0), size=(1.0, 1.0, 1.0)) -> np.ndarray:
    """Return outward-wound triangles for an axis-aligned rectangular solid."""
    vertices = np.array(
        [
            (0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
            (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1),
        ],
        dtype=float,
    )
    vertices = vertices * np.asarray(size) + np.asarray(origin)
    faces = np.array(
        [
            (0, 2, 1), (0, 3, 2),  # bottom
            (4, 5, 6), (4, 6, 7),  # top
            (0, 1, 5), (0, 5, 4),  # front
            (3, 7, 6), (3, 6, 2),  # back
            (0, 4, 7), (0, 7, 3),  # left
            (1, 2, 6), (1, 6, 5),  # right
        ]
    )
    return vertices[faces]


def tetra_triangles() -> np.ndarray:
    """Axis lengths 1, 2, 3 m give exactly 1 m³ of volume."""
    vertices = np.array(
        [(0, 0, 0), (1, 0, 0), (0, 2, 0), (0, 0, 3)], dtype=float
    )
    faces = np.array([(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)])
    return vertices[faces]


class SolidPropertiesTest(unittest.TestCase):
    def test_unit_cube_analytic_volume_centroid_and_inertia(self) -> None:
        properties = solid_properties(cube_triangles().reshape(-1, 3))
        self.assertEqual(properties["closed_shells"], 1)
        self.assertAlmostEqual(properties["volume_m3"], 1.0)
        np.testing.assert_allclose(properties["centroid_m"], [0.5, 0.5, 0.5], atol=1e-12)
        # Unit density gives unit mass; each central principal moment is m/6.
        np.testing.assert_allclose(
            properties["inertia_unit_density_m5"], np.eye(3) / 6, atol=1e-12
        )

    def test_tetrahedron_analytic_volume_centroid_and_inertia(self) -> None:
        properties = solid_properties(tetra_triangles().reshape(-1, 3))
        self.assertAlmostEqual(properties["volume_m3"], 1.0)
        np.testing.assert_allclose(properties["centroid_m"], [0.25, 0.5, 0.75], atol=1e-12)
        # Uniform tetrahedron covariance: Var(x_i)=3*a_i²/80,
        # Cov(x_i,x_j)=-a_i*a_j/80. Inertia is tr(Cov)I-Cov for unit mass.
        expected = np.array(
            [[39 / 80, 2 / 80, 3 / 80],
             [2 / 80, 30 / 80, 6 / 80],
             [3 / 80, 6 / 80, 15 / 80]]
        )
        np.testing.assert_allclose(properties["inertia_unit_density_m5"], expected, atol=1e-12)

    def test_open_and_reversed_surfaces_are_rejected(self) -> None:
        cube = cube_triangles()
        with self.assertRaisesRegex(ValueError, "watertight"):
            solid_properties(cube[:-1].reshape(-1, 3))
        partly_reversed = cube.copy()
        partly_reversed[0] = partly_reversed[0][::-1]
        with self.assertRaisesRegex(ValueError, "winding"):
            solid_properties(partly_reversed.reshape(-1, 3))
        with self.assertRaisesRegex(ValueError, "reversed connected shell"):
            solid_properties(cube[:, ::-1, :].reshape(-1, 3))

    def test_disjoint_solids_are_both_counted(self) -> None:
        left = cube_triangles()
        right = cube_triangles(origin=(2.0, 0.0, 0.0))
        properties = solid_properties(np.concatenate((left, right)).reshape(-1, 3))
        self.assertEqual(properties["closed_shells"], 2)
        np.testing.assert_allclose(properties["shell_volumes_m3"], [1, 1], atol=1e-12)
        self.assertAlmostEqual(properties["volume_m3"], 2.0)
        np.testing.assert_allclose(properties["centroid_m"], [1.5, 0.5, 0.5], atol=1e-12)
        np.testing.assert_allclose(
            properties["inertia_unit_density_m5"],
            np.diag([1 / 3, 7 / 3, 7 / 3]),
            atol=1e-12,
        )

    def test_rotation_translation_and_parallel_axis_aggregation(self) -> None:
        first = solid_properties(tetra_triangles().reshape(-1, 3))
        rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        translation = np.array([2.0, -1.0, 0.5])
        moved_vertices = tetra_triangles() @ rotation.T + translation
        second = solid_properties(moved_vertices.reshape(-1, 3))
        first_center = np.array([0.25, 0.5, 0.75])
        first_inertia = np.array(
            [[39 / 80, 2 / 80, 3 / 80],
             [2 / 80, 30 / 80, 6 / 80],
             [3 / 80, 6 / 80, 15 / 80]]
        )
        second_center = rotation @ first_center + translation
        np.testing.assert_allclose(second["centroid_m"], second_center, atol=1e-12)
        np.testing.assert_allclose(
            second["inertia_unit_density_m5"], rotation @ first_inertia @ rotation.T, atol=1e-12
        )

        items = [
            {"volume_m3": 1.0, "com_source_m": first_center,
             "inertia_unit_density_source_m5": first_inertia},
            {"volume_m3": 1.0, "com_source_m": second_center,
             "inertia_unit_density_source_m5": rotation @ first_inertia @ rotation.T},
        ]
        result = aggregate_mesh_group(items, [2.0, 3.0])
        expected_center = (2 * first_center + 3 * second_center) / 5
        expected_inertia = np.zeros((3, 3))
        for mass, center, tensor in (
            (2.0, first_center, first_inertia),
            (3.0, second_center, rotation @ first_inertia @ rotation.T),
        ):
            displacement = center - expected_center
            expected_inertia += mass * (
                tensor + np.dot(displacement, displacement) * np.eye(3)
                - np.outer(displacement, displacement)
            )
        self.assertAlmostEqual(result["mass_kg"], 5.0)
        np.testing.assert_allclose(result["com_source_m"], expected_center, atol=1e-12)
        np.testing.assert_allclose(
            result["inertia_about_com_source_kg_m2"], expected_inertia, atol=1e-12
        )

    def test_aggregation_rejects_missing_or_invalid_mass_assignments(self) -> None:
        properties = solid_properties(cube_triangles().reshape(-1, 3))
        item = {
            "volume_m3": properties["volume_m3"],
            "com_source_m": properties["centroid_m"],
            "inertia_unit_density_source_m5": properties["inertia_unit_density_m5"],
        }
        for items, masses in (
            ([], []),
            ([item, item], [1.0]),
            ([item], []),
            ([item], [0.0]),
            ([item], [-1.0]),
            ([item], [float("nan")]),
        ):
            with self.subTest(items=len(items), masses=masses):
                with self.assertRaisesRegex(ValueError, "positive assigned mass"):
                    aggregate_mesh_group(items, masses)


class GeneratedAuditTest(unittest.TestCase):
    def test_saved_audit_mass_balance_and_static_markers(self) -> None:
        audit_path = PROJECT / "simulation/models/wall_e_mass_audit.json"
        marker_path = PROJECT / "simulation/models/wall_e_com_inspection.xml"
        if not audit_path.is_file() or not marker_path.is_file():
            self.skipTest("Mass audit artifacts have not been generated yet")
        report = json.loads(audit_path.read_text(encoding="utf-8"))
        self.assertAlmostEqual(report["battery_group"]["mass_kg"], 0.153, places=12)
        self.assertEqual([part["closed_shells"] for part in report["battery_group"]["parts"]], [3, 1, 6])
        self.assertAlmostEqual(report["screws"]["mass_kg"], 4 * 0.0045, places=12)
        self.assertEqual(len(report["screws"]["parts"]), 4)
        for part in report["screws"]["parts"]:
            self.assertAlmostEqual(part["mass_kg"], 0.0045, places=12)
        structural = report["structural_parts_group"]
        self.assertEqual([part["link"] for part in structural["parts"]], ["part_1", "part_1_1", "part_2"])
        self.assertAlmostEqual(structural["mass_kg"], 0.04084, places=12)
        self.assertEqual({part["link"]: part["mass_kg"] for part in structural["parts"]},
                         {"part_1": 0.032, "part_1_1": 0.00442, "part_2": 0.00442})
        self.assertAlmostEqual(report["scenarios"]["pcb_30g"]["pcb_mass_kg"], 0.030, places=12)

        display = np.array(report["display_translation_m"])
        for scenario in report["scenarios"].values():
            wheels = list(scenario["wheel_groups"].values())
            self.assertEqual(len(wheels), 2)
            for wheel in wheels:
                self.assertAlmostEqual(wheel["mass_kg"], 0.040, places=9)
            masses = [scenario["chassis_mass_kg"]] + [wheel["mass_kg"] for wheel in wheels]
            centers = [scenario["chassis_com_source_m"]] + [wheel["com_source_m"] for wheel in wheels]
            self.assertAlmostEqual(sum(masses), scenario["assembly_mass_kg"], places=12)
            weighted_center = sum(
                (mass * np.asarray(center) for mass, center in zip(masses, centers)),
                np.zeros(3),
            ) / sum(masses)
            np.testing.assert_allclose(weighted_center, scenario["assembly_com_source_m"], atol=1e-12)
            np.testing.assert_allclose(
                np.asarray(scenario["chassis_com_source_m"]) + display,
                scenario["chassis_com_display_m"], atol=1e-12,
            )

        model = ET.parse(marker_path).getroot()
        self.assertFalse(model.findall(".//inertial"))
        sites = {site.get("name") for site in model.findall(".//site")}
        self.assertTrue(
            {"chassis_com_assumption_based", "assembly_com_assumption_based", "battery_group_com"}
            <= sites
        )


if __name__ == "__main__":
    unittest.main()
