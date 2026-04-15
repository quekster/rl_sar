#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float):
    """Return world-from-body rotation matrix from quaternion."""
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def rotate_world_to_body(rot_wb, vec_w):
    """Apply R_bw = R_wb^T."""
    return (
        rot_wb[0][0] * vec_w[0] + rot_wb[1][0] * vec_w[1] + rot_wb[2][0] * vec_w[2],
        rot_wb[0][1] * vec_w[0] + rot_wb[1][1] * vec_w[1] + rot_wb[2][1] * vec_w[2],
        rot_wb[0][2] * vec_w[0] + rot_wb[1][2] * vec_w[1] + rot_wb[2][2] * vec_w[2],
    )


class GazeboMockOdomBridge(Node):
    def __init__(self):
        super().__init__("gazebo_mock_odom_bridge")

        self.declare_parameter("gazebo_model_name", "go2_gazebo")
        self.declare_parameter("input_topic", "/gazebo/model_states")
        self.declare_parameter("output_topic", "/ekf/input/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")

        self.model_name = self.get_parameter("gazebo_model_name").get_parameter_value().string_value
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        self.model_index: Optional[int] = None
        self.missing_model_warn_count = 0

        self.pub = self.create_publisher(Odometry, self.output_topic, 20)
        self.sub = self.create_subscription(ModelStates, self.input_topic, self.model_states_cb, 20)

        self.get_logger().info(
            f"Bridging {self.input_topic} -> {self.output_topic} for model '{self.model_name}' "
            f"(frame_id={self.odom_frame}, child_frame_id={self.base_frame})"
        )

    def _lookup_model_index(self, msg: ModelStates) -> Optional[int]:
        if self.model_index is not None and self.model_index < len(msg.name):
            if msg.name[self.model_index] == self.model_name:
                return self.model_index

        try:
            self.model_index = msg.name.index(self.model_name)
            self.get_logger().info(f"Found model '{self.model_name}' at index {self.model_index}")
            return self.model_index
        except ValueError:
            self.model_index = None
            return None

    def model_states_cb(self, msg: ModelStates):
        idx = self._lookup_model_index(msg)
        if idx is None:
            self.missing_model_warn_count += 1
            if self.missing_model_warn_count % 50 == 0:
                names_preview = ", ".join(msg.name[:6])
                self.get_logger().warn(
                    f"Model '{self.model_name}' not found in /gazebo/model_states. "
                    f"Example names: [{names_preview}]"
                )
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if q_norm < 1e-9:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            qx, qy, qz, qw = qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm

        rot_wb = quat_to_rot_matrix(qx, qy, qz, qw)
        linear_b = rotate_world_to_body(rot_wb, (twist.linear.x, twist.linear.y, twist.linear.z))
        angular_b = rotate_world_to_body(rot_wb, (twist.angular.x, twist.angular.y, twist.angular.z))

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose = pose
        odom.twist.twist.linear.x = linear_b[0]
        odom.twist.twist.linear.y = linear_b[1]
        odom.twist.twist.linear.z = linear_b[2]
        odom.twist.twist.angular.x = angular_b[0]
        odom.twist.twist.angular.y = angular_b[1]
        odom.twist.twist.angular.z = angular_b[2]

        # Conservative defaults; tune once real estimator is swapped in.
        odom.pose.covariance = [
            10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 10.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 99.0,
        ]
        odom.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.10, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.10, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.10,
        ]

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboMockOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
