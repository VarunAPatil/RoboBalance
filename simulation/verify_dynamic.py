"""Verify the compiled provisional Wall-E dynamics model and static isolation."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from simulation.build_dynamic import AUDIT, MASS_AUDIT, OUTPUT, PREVIEW, PREVIEW_AUDIT, ROOT


def body_id(model: mujoco.MjModel, name: str) -> int:
    value = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert value >= 0, name
    return value


def geom_id(model: mujoco.MjModel, name: str) -> int:
    value = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert value >= 0, name
    return value


def compiled_full_inertia(model: mujoco.MjModel, index: int) -> np.ndarray:
    rotation_flat = np.zeros(9)
    mujoco.mju_quat2Mat(rotation_flat, model.body_iquat[index])
    rotation = rotation_flat.reshape(3, 3)
    return rotation @ np.diag(model.body_inertia[index]) @ rotation.T


def main() -> None:
    audit = json.loads(AUDIT.read_text())
    mass = json.loads(MASS_AUDIT.read_text())
    preview_audit = json.loads(PREVIEW_AUDIT.read_text())
    source = ROOT / audit["source_urdf"]
    assert ET.parse(OUTPUT).getroot().find("custom") is None
    assert hashlib.sha256(source.read_bytes()).hexdigest() == audit["source_sha256"]
    assert audit["selected_mass_scenario"] == "pcb_reconstructed"
    assert audit["source_sha256"] == mass["source_sha256"] == preview_audit["source_sha256"]
    model = mujoco.MjModel.from_xml_path(str(OUTPUT))
    static = mujoco.MjModel.from_xml_path(str(PREVIEW))
    data = mujoco.MjData(model)
    static_data = mujoco.MjData(static)
    mujoco.mj_forward(model, data)
    mujoco.mj_forward(static, static_data)
    assert (model.nbody, model.njnt, model.nq, model.nv, model.nu) == (4, 3, 9, 8, 0)
    assert np.allclose(model.opt.gravity, [0, 0, -9.81])
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qacc).all()
    assert np.isfinite(data.xpos).all() and np.isfinite(data.geom_xpos).all()
    # The balance/PID loop must not begin during a free-fall gap.  Check both
    # wheel proxies are in near-zero-distance contact with the floor at qpos=0.
    floor = geom_id(model, "floor_collision_and_visual")
    for wheel in ("wheel_left_collision", "wheel_right_collision"):
        tire = geom_id(model, wheel)
        distance = mujoco.mj_geomDistance(model, data, tire, floor, 0.01, None)
        assert -2e-5 < distance <= 0.0, f"{wheel} initial floor distance is {distance:.9g} m"
    assert data.ncon > 0, "Robot must start with tire-floor contacts, not fall onto the floor"
    expected_mass = mass["scenarios"]["pcb_reconstructed"]["assembly_mass_kg"]
    np.testing.assert_allclose(model.body_mass.sum(), expected_mass, rtol=0, atol=1e-12)

    for label, name in (("chassis", "chassis"), ("left", "wheel_left"), ("right", "wheel_right")):
        index = body_id(model, name)
        entry = audit["body_properties"][label]
        np.testing.assert_allclose(model.body_mass[index], entry["mass_kg"], rtol=0, atol=1e-13)
        np.testing.assert_allclose(model.body_ipos[index], entry["com_body_m"], rtol=0, atol=1e-10)
        np.testing.assert_allclose(compiled_full_inertia(model, index),
                                   entry["inertia_body_about_com_kg_m2"], rtol=1e-8, atol=5e-11)
        principal = np.linalg.eigvalsh(compiled_full_inertia(model, index))
        assert np.all(principal > 0) and principal[-1] <= principal[0] + principal[1]

    for i, (label, name) in enumerate((("left", "wheel_left_hinge"),
                                       ("right", "wheel_right_hinge")), start=1):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert joint == i and model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_HINGE
        frame = audit["hinge_urdf_frames"][label]
        np.testing.assert_allclose(model.jnt_axis[joint], frame["axis_local"], atol=1e-12)
        parent = body_id(model, f"wheel_{label}")
        world_axis = data.xmat[parent].reshape(3, 3) @ model.jnt_axis[joint]
        np.testing.assert_allclose(world_axis, frame["axis_source"], atol=1e-8)
        source_hinge = data.xpos[parent] - np.array(audit["initial_chassis_translation_source_to_world_m"])
        np.testing.assert_allclose(source_hinge, frame["origin_source_m"], atol=1e-9)

    active = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
              for i in range(model.ngeom) if model.geom_contype[i] or model.geom_conaffinity[i]}
    assert active == {"floor_collision_and_visual", "wheel_left_collision", "wheel_right_collision"}
    dynamic_visuals = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i): i
                       for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH}
    static_visuals = {mujoco.mj_id2name(static, mujoco.mjtObj.mjOBJ_GEOM, i): i
                      for i in range(static.ngeom) if static.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH}
    assert dynamic_visuals.keys() == static_visuals.keys()
    assert len(dynamic_visuals) == preview_audit["visual_instances"] == 404
    delta = np.array(audit["initial_chassis_translation_source_to_world_m"]) - np.array(
        preview_audit["display_translation_m"])
    for name, index in dynamic_visuals.items():
        reference = static_visuals[name]
        np.testing.assert_allclose(data.geom_xpos[index] - static_data.geom_xpos[reference],
                                   delta, atol=1e-9)
        np.testing.assert_allclose(data.geom_xmat[index], static_data.geom_xmat[reference], atol=1e-9)
        assert model.geom_contype[index] == model.geom_conaffinity[index] == 0

    left_rim = geom_id(model, "visual_rim_1_0")
    right_rim = geom_id(model, "visual_rim_0")
    left_before = data.geom_xmat[left_rim].copy()
    right_before = data.geom_xmat[right_rim].copy()
    chassis_before = data.xmat[body_id(model, "chassis")].copy()
    data.qpos[7] = 0.4
    mujoco.mj_forward(model, data)
    assert np.linalg.norm(data.geom_xmat[left_rim] - left_before) > 0.1
    np.testing.assert_allclose(data.geom_xmat[right_rim], right_before, atol=1e-12)
    np.testing.assert_allclose(data.xmat[body_id(model, "chassis")], chassis_before, atol=1e-12)
    data.qpos[7] = 0
    data.qpos[8] = 0.4
    mujoco.mj_forward(model, data)
    assert np.linalg.norm(data.geom_xmat[right_rim] - right_before) > 0.1
    np.testing.assert_allclose(data.geom_xmat[left_rim], left_before, atol=1e-12)

    # This checks that gravity/contact are connected; it does not validate
    # balancing, physical friction, the chassis collision shape, or motors.
    falling = mujoco.MjData(model)
    for _ in range(50):
        mujoco.mj_step(model, falling)
    assert falling.time > 0 and falling.ncon > 0
    assert np.isfinite(falling.qpos).all() and np.isfinite(falling.qvel).all()
    print(f"MuJoCo {mujoco.__version__}: mass {model.body_mass.sum():.12f} kg; "
          f"3 joints/8 DOF, 0 actuators, {len(active)} active collision geoms; "
          f"{len(dynamic_visuals)} preserved visuals")
    print("Compiled inertias/COMs, URDF hinge frames, wheel isolation, mj_forward, gravity/contact, and static-geometry preservation verified")


if __name__ == "__main__":
    main()
