#!/usr/bin/env python3

import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


@dataclass
class RobotData:
    ee_pos: np.ndarray # (3,) [tx, ty, tz]
    ee_quat: np.ndarray # (4,) [qw, qx, qy, qz]
    posture: np.ndarray # (7,) [j1-j7]
    ee_compliance: np.ndarray # (6,) [rx, ry, rz, tx, ty, tz]
    posture_compliance: np.ndarray # (7,) [j1-j7]
    gripper_opening: np.ndarray # (1,)


def _as_numpy(name: str, value: np.ndarray, size: int) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {array.shape}")
    return array


def _pack_robot_data(
    ee_pos: np.ndarray,
    ee_quat: np.ndarray,
    posture: np.ndarray,
    ee_compliance: np.ndarray,
    posture_compliance: np.ndarray,
    gripper_opening: np.ndarray,
) -> list:
    ee_pos = _as_numpy("ee_pos", ee_pos, 3)
    ee_quat = _as_numpy("ee_quat", ee_quat, 4)
    posture = _as_numpy("posture", posture, 7)
    ee_compliance = _as_numpy("ee_compliance", ee_compliance, 6)
    posture_compliance = _as_numpy("posture_compliance", posture_compliance, 7)
    gripper_opening = _as_numpy("gripper_opening", gripper_opening, 1)
    return np.concatenate(
        [ee_pos, ee_quat, posture, ee_compliance, posture_compliance, gripper_opening]
    ).tolist()


def _unpack_robot_data(data: np.ndarray) -> RobotData:
    array = np.asarray(data, dtype=np.float64)
    if array.shape != (28,):
        raise ValueError(f"robot data must have shape (28,), got {array.shape}")
    return RobotData(
        ee_pos=array[0:3].copy(),
        ee_quat=array[3:7].copy(),
        posture=array[7:14].copy(),
        ee_compliance=array[14:20].copy(),
        posture_compliance=array[20:27].copy(),
        gripper_opening=array[27:28].copy(),
    )


class ExplicitCompClient(Node):
    def __init__(
        self,
        node_name: str = "explicit_comp_client",
        command_topic: str = "robot_command_data",
        measured_topic: str = "robot_measured_data",
    ) -> None:
        super().__init__(node_name)
        self._publisher = self.create_publisher(Float64MultiArray, command_topic, 1)
        self._latest_measured: Optional[RobotData] = None
        self._subscription = self.create_subscription(
            Float64MultiArray, measured_topic, self._measured_callback, 1
        )

    def _measured_callback(self, msg: Float64MultiArray) -> None:
        self._latest_measured = _unpack_robot_data(np.asarray(msg.data, dtype=np.float64))

    def send_command(
        self,
        *,
        ee_pos: np.ndarray,
        ee_quat: np.ndarray,
        posture: np.ndarray,
        ee_compliance: np.ndarray,
        posture_compliance: np.ndarray,
        gripper_opening: np.ndarray,
    ) -> None:
        msg = Float64MultiArray()
        msg.data = _pack_robot_data(
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            posture=posture,
            ee_compliance=ee_compliance,
            posture_compliance=posture_compliance,
            gripper_opening=gripper_opening,
        )
        self._publisher.publish(msg)

    def spin_until_measured(
        self, timeout_sec: Optional[float] = None, spin_timeout_sec: float = 0.1
    ) -> RobotData:
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=spin_timeout_sec)
            if self._latest_measured is not None:
                return self._latest_measured
            if timeout_sec is not None and time.monotonic() - start >= timeout_sec:
                raise TimeoutError("Timed out waiting for robot_measured_data")
        raise RuntimeError("rclpy shutdown before receiving robot_measured_data")

    def spin_measured_loop(
        self,
        callback: Callable[[RobotData], None],
        spin_timeout_sec: float = 0.1,
    ) -> None:
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=spin_timeout_sec)
            if self._latest_measured is not None:
                callback(self._latest_measured)

    @property
    def latest_measured(self) -> Optional[RobotData]:
        return self._latest_measured


def create_client(
    node_name: str = "explicit_comp_client",
    command_topic: str = "robot_command_data",
    measured_topic: str = "robot_measured_data",
) -> ExplicitCompClient:
    if not rclpy.ok():
        rclpy.init()
    return ExplicitCompClient(
        node_name=node_name,
        command_topic=command_topic,
        measured_topic=measured_topic,
    )


def shutdown_client(client: ExplicitCompClient) -> None:
    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    client = create_client()
    try:
        client.send_command(
            ee_pos=np.array([0.6, 0.0, 0.5]),
            ee_quat=np.array([-0.5, 0.5, 0.5, 0.5]),
            posture=np.array([0.0, 0.262, 3.14, -2.269, 0.0, 0.96, 1.57]),
            ee_compliance=np.ones(6),
            posture_compliance=np.ones(7),
            gripper_opening=np.array([1.0]),
        )
        measured = client.spin_until_measured(timeout_sec=1.0)
        print("ee_pos:", measured.ee_pos)
        print("ee_quat:", measured.ee_quat)
        print("posture:", measured.posture)
        print("ee_compliance:", measured.ee_compliance)
        print("posture_compliance:", measured.posture_compliance)
        print("gripper_opening:", measured.gripper_opening)
    finally:
        shutdown_client(client)
