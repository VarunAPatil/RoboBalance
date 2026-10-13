"""Compile and audit the generated geometry-only Wall-E preview."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from simulation.convert_urdf import DEFAULT_SOURCE, read_stl, signed_volume
from simulation.preview import DEFAULT_MODEL


def closed_components(vertices: np.ndarray) -> int:
    """Count disconnected closed solids using exact STL vertex/edge incidence."""
    _, inverse = np.unique(vertices, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    sorted_edges = np.sort(edges, axis=1)
    _, edge_inverse = np.unique(sorted_edges, axis=0, return_inverse=True)
    count = np.bincount(edge_inverse)
    orientation = np.bincount(edge_inverse, weights=np.where(edges[:, 0] < edges[:, 1], 1, -1))
    if not np.all(count == 2) or not np.all(orientation == 0):
        raise AssertionError("Key weighed mesh is not closed and consistently wound")
    parent = list(range(int(faces.max()) + 1))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for a, b, c in faces:
        ra, rb, rc = find(int(a)), find(int(b)), find(int(c))
        parent[rb] = ra
        parent[rc] = ra
    return len({find(index) for index in range(len(parent))})


def main() -> None:
    report_path = DEFAULT_MODEL.with_name("wall_e_preview_audit.json")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    project = Path(__file__).resolve().parents[1]
    source = project / report["source_urdf"]
    assert source.resolve() == DEFAULT_SOURCE.resolve(), "Preview was generated from the wrong assembly"
    if hashlib.sha256(source.read_bytes()).hexdigest() != report["source_sha256"]:
        raise AssertionError("Source URDF changed after conversion; regenerate the preview")
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.nbody == 4 and model.njnt == 0
    assert model.nmesh == report["unique_mesh_assets"]
    assert model.ngeom == report["visual_instances"] + 1
    assert model.ncam == 2
    assert np.all(model.body_mass == 0) and np.all(model.body_inertia == 0), "Static visuals must not infer physical mass"
    assert np.all(model.geom_contype == 0) and np.all(model.geom_conaffinity == 0)
    body_counts = {}
    for body in ("chassis_static", "wheel_left_static", "wheel_right_static"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        assert body_id >= 0
        body_counts[body] = int(np.count_nonzero(model.geom_bodyid == body_id))
    assert sorted(body_counts.values()) == [2, 2, report["visual_instances"] - 4], body_counts
    key = report["key_visuals"]
    assert len(key["screws"]) == 4 and len(key["rims"]) == 2 and len(key["tires"]) == 2
    assert len(key["motor_shells"]) == len(key["motor_cores"]) == len(key["motor_shafts"]) == 2
    assert all(len(key[name]) == 1 for name in ("battery_cells_mesh", "battery_holder_mesh", "battery_springs_mesh", "main_pcb_mesh", "daughterboard_mesh"))
    for entries in key.values():
        for item in entries:
            geom_name = f"visual_{item['link']}_0"
            assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name) >= 0, geom_name

    # MuJoCo may recenter mesh assets internally. Verify world-space vertices
    # after its compiled geom transforms, without adding a manual correction.
    minima, maxima = [], []
    for geom_id in range(model.ngeom):
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_id = model.geom_dataid[geom_id]
        start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        vertices = model.mesh_vert[start:start + count]
        world_vertices = vertices @ data.geom_xmat[geom_id].reshape(3, 3).T + data.geom_xpos[geom_id]
        minima.append(world_vertices.min(axis=0))
        maxima.append(world_vertices.max(axis=0))
    actual_min, actual_max = np.min(minima, axis=0), np.max(maxima, axis=0)
    expected_min = np.array(report["display_bounds_m"]["min"])
    expected_max = np.array(report["display_bounds_m"]["max"])
    np.testing.assert_allclose(actual_min, expected_min, atol=1e-5, rtol=0)
    np.testing.assert_allclose(actual_max, expected_max, atol=1e-5, rtol=0)
    assert 0 < actual_min[2] < 0.005
    for hinge in report["hinges_inspection_only"].values():
        axis = np.array(hinge["axis_in_source_world"])
        np.testing.assert_allclose(np.linalg.norm(axis), 1, atol=1e-6)
    mesh_dir = source.parent.parent / "meshes"
    solids_expected = {
        "Battery18650__NONE_2.stl": 3,
        "BatteryCasePlastic__Fillet1.stl": 1,
        "BatteryCaseSpring__Cut_Extrude2.stl": 6,
        "Screw_M3_75mm.stl": 1,
    }
    for filename, expected_count in solids_expected.items():
        vertices = read_stl(mesh_dir / filename)
        assert closed_components(vertices) == expected_count, filename
        assert signed_volume(vertices) > 0, filename
    print(f"Closed component counts verified: {solids_expected}")
    print(f"Compiled MuJoCo {mujoco.__version__}: {model.nbody - 1} fixed bodies, {model.ngeom - 1} visual meshes, {model.nmesh} assets")
    print(f"Body visual counts: {body_counts}")
    print(f"Display bounds (m): min={actual_min.round(6)}, max={actual_max.round(6)}")
    print("Key wheel, motor, PCB, battery, holder, spring, and screw geoms present; source hash and world placement verified")


if __name__ == "__main__":
    main()
