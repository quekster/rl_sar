#!/usr/bin/env python3
# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import math
import struct
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray

import tf2_ros
from tf2_ros import TransformException


def _quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    # converts a quaternion (qx, qy, qz, qw) into a 3×3 rotation matrix.
    # TF transform gives translation + quaternion rotation, so need rotation matrix to change frame
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)

    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)

    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)

    return ((r00, r01, r02), (r10, r11, r12), (r20, r21, r22))


def _transform_point(
    # converts each point from lidar_link frame to base frame, using the translation and rotation from TF lookup
    point: Tuple[float, float, float],
    translation: Tuple[float, float, float],
    rotation: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]],
) -> Tuple[float, float, float]:
    px, py, pz = point
    tx, ty, tz = translation

    rx = rotation[0][0] * px + rotation[0][1] * py + rotation[0][2] * pz
    ry = rotation[1][0] * px + rotation[1][1] * py + rotation[1][2] * pz
    rz = rotation[2][0] * px + rotation[2][1] * py + rotation[2][2] * pz

    return (rx + tx, ry + ty, rz + tz)


class LidarObsPreprocessor(Node):
    def __init__(self) -> None:
        super().__init__("lidar_obs_preprocessor")

        self.declare_parameter("input_topic", "/rl_sar/lidar_points")
        self.declare_parameter("output_topic", "/rl_sar/lidar_obs_flat")
        self.declare_parameter("target_frame", "base")
        self.declare_parameter("expected_points", 45)
        self.declare_parameter("max_range", 70.0)
        self.declare_parameter("stale_warn_s", 0.2)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.expected_points = int(self.get_parameter("expected_points").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.stale_warn_s = float(self.get_parameter("stale_warn_s").value)

        self.publisher = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._pointcloud_callback,
            qos_profile_sensor_data,
        ) ##Subscribes PointCloud2 from /rl_sar/lidar_points

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_msg_time = None
        self.msg_count_window = 0
        self.nan_count_window = 0
        self.diag_cycle = 0
        self.create_timer(1.0, self._diag_timer_callback)

        self.get_logger().info(
            f"Lidar preprocessor listening on {self.input_topic}, publishing {self.output_topic}, "
            f"target_frame={self.target_frame}, expected_points={self.expected_points}, max_range={self.max_range}"
        )

    def _field_offsets(self, msg: PointCloud2):
        # finds the byte offsets of x/y/z fields in the PointCloud2 message (binary), which are needed to read the point data correctly.
        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == "x":
                x_off = field.offset
            elif field.name == "y":
                y_off = field.offset
            elif field.name == "z":
                z_off = field.offset
        return x_off, y_off, z_off

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        self.last_msg_time = self.get_clock().now()
        self.msg_count_window += 1

        x_off, y_off, z_off = self._field_offsets(msg)
        if x_off is None or y_off is None or z_off is None:
            self.get_logger().warn("PointCloud2 missing x/y/z fields")
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed ({msg.header.frame_id} -> {self.target_frame}): {ex}")
            return

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        translation = (t.x, t.y, t.z)
        rotation = _quat_to_rot_matrix(q.x, q.y, q.z, q.w)

        point_count = msg.width * msg.height
        endian = ">" if msg.is_bigendian else "<"
        fmt = endian + "f"

        transformed_points: List[Tuple[float, float, float]] = []
        for i in range(point_count):
            base = i * msg.point_step
            x = struct.unpack_from(fmt, msg.data, base + x_off)[0]
            y = struct.unpack_from(fmt, msg.data, base + y_off)[0]
            z = struct.unpack_from(fmt, msg.data, base + z_off)[0]

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                self.nan_count_window += 1
                transformed_points.append((self.max_range, self.max_range, self.max_range))
                continue

            transformed_points.append(_transform_point((x, y, z), translation, rotation))

        if len(transformed_points) < self.expected_points:
            pad_count = self.expected_points - len(transformed_points)
            transformed_points.extend([(self.max_range, self.max_range, self.max_range)] * pad_count)
        elif len(transformed_points) > self.expected_points:
            transformed_points = transformed_points[: self.expected_points]

        flat: List[float] = []
        inv_range = 1.0 / self.max_range
        for px, py, pz in transformed_points:
            flat.append(float(px * inv_range))
            flat.append(float(py * inv_range))
            flat.append(float(pz * inv_range))

        out = Float32MultiArray()
        out.data = flat
        self.publisher.publish(out)

    def _diag_timer_callback(self) -> None:
        now = self.get_clock().now()

        if self.last_msg_time is not None:
            stale_s = (now - self.last_msg_time).nanoseconds * 1e-9
            if stale_s > self.stale_warn_s:
                self.get_logger().warn(
                    f"Input lidar stale: {stale_s:.3f}s (> {self.stale_warn_s:.3f}s)"
                )

        self.diag_cycle += 1
        if self.diag_cycle % 5 == 0:
            self.get_logger().info(
                f"diag: rate={self.msg_count_window:.1f}Hz over 1s, nan_count={self.nan_count_window}, "
                f"published_len={self.expected_points * 3}"
            )

        self.msg_count_window = 0
        self.nan_count_window = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarObsPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
