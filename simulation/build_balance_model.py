"""Build a separate sensor/actuator MJCF from the provisional dynamics XML."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

from simulation.balance_config import DEFAULT_CONFIG, load_config, require_imu_pose
from simulation.build_dynamic import OUTPUT as DYNAMIC_MODEL


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "simulation/models/wall_e_balance.xml"


def build_balance_model(
    config: dict,
    source: Path = DYNAMIC_MODEL,
    output: Path = DEFAULT_OUTPUT,
) -> Path:
    source = Path(source).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    if output == source:
        raise ValueError("Balance MJCF must be a separate file; refusing to overwrite the dynamics source")
    if output.parent != source.parent:
        raise ValueError(
            "Balance MJCF must be written beside the source model so its relative mesh asset paths remain valid"
        )
    pos, quat = require_imu_pose(config)
    tree = ET.parse(source)
    model = tree.getroot()
    if model.find(".//site[@name='imu_site']") is not None:
        raise ValueError("Source already contains imu_site; refusing duplicate sensor frame")
    if model.find("actuator") is not None or model.find("sensor") is not None:
        raise ValueError("Source already has actuator/sensor sections; refusing duplicate definitions")

    chassis = model.find("./worldbody/body[@name='chassis']")
    if chassis is None:
        raise ValueError("Dynamic MJCF has no worldbody chassis")
    ET.SubElement(chassis, "site", name="imu_site",
                  pos=" ".join(f"{value:.16g}" for value in pos),
                  quat=" ".join(f"{value:.16g}" for value in quat),
                  size="0.003", rgba="0 1 0 1")

    actuator = ET.SubElement(model, "actuator")
    # gear=1 means control is hinge torque in N*m. The Python motor model maps
    # firmware PWM to torque; these are not electrical motor models.
    ET.SubElement(actuator, "motor", name="left_wheel_torque",
                  joint="wheel_left_hinge", gear="1")
    ET.SubElement(actuator, "motor", name="right_wheel_torque",
                  joint="wheel_right_hinge", gear="1")
    sensor = ET.SubElement(model, "sensor")
    ET.SubElement(sensor, "accelerometer", name="imu_accel", site="imu_site")
    ET.SubElement(sensor, "gyro", name="imu_gyro", site="imu_site")
    model.set("model", "wall_e_balance_sensor_actuator_scaffold")
    model.insert(0, ET.Comment(
        "Derived scaffold. IMU pose is user-configured; motor actuators accept torque, "
        "not raw PWM. No hardware motor parameters are assumed."))
    ET.indent(tree, space="  ")
    tree.write(output, encoding="unicode", xml_declaration=True)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--source", type=Path, default=DYNAMIC_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    config = load_config(args.config)
    result = build_balance_model(config, args.source, args.output)
    print(f"Built sensor/actuator scaffold: {result}")
    print("This does not validate the configured IMU pose or motor dynamics.")


if __name__ == "__main__":
    main()
