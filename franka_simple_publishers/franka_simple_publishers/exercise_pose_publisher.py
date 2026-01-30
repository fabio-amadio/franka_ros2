#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class ExercisePosePub(Node):
    def __init__(self):
        super().__init__("exercise_pose_pub")

        # Publisher
        self.pub = self.create_publisher(
            PoseStamped, "/cartesian_impedance/desired_pose", 10
        )

        # Subscriber (we only use the first message as start pose)
        self.sub = self.create_subscription(
            PoseStamped,
            "/cartesian_impedance/current_pose",
            self.current_pose_cb,
            10
        )

        ''' --- TODO: Declare, load and process the "goal" parameter --- '''

        # Parameter: goal = [T, x, y, z, qx, qy, qz, qw]
        self.declare_parameter(
            "goal", [5.0, 0.3, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
        )
        goal = np.array(self.get_parameter("goal").value)

        if len(goal) != 8:
            raise ValueError(
                "'goal' must have 8 values: [T, x, y, z, qx, qy, qz, qw]"
            )

        self.T = goal[0]

        if self.T <= 0.0:
            raise ValueError("Trajectory duration T must be > 0")
        
        self.goal_pos = goal[1:4]
        self.goal_quat = goal[4:8]

        quat_norm = np.linalg.norm(self.goal_quat)
        if quat_norm < 1e-9:
            raise ValueError("Goal quaternion has near-zero norm")

        self.goal_quat = self.goal_quat / quat_norm

        ''' ----------------------------------------------------------- '''

        # Filled when we receive the first current pose
        self.start_pos = None
        self.start_quat = None
        self.slerp = None

        # Timing
        self.t = 0

        # Publish rate
        self.rate_hz = 20.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_cb)

        self.get_logger().info("Node started. Waiting for current pose ...")

    def current_pose_cb(self, msg: PoseStamped):
        # We need to read the pose only once
        if self.start_pos is not None:
            return

        self.start_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

        self.start_quat = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
        )
        self.start_quat = self.start_quat / np.linalg.norm(self.start_quat)

        self.get_logger().info("Got start pose. Start publishing trajectory.")

    def timer_cb(self):
        # Do nothing if we didn't yet received the starting pose
        if self.start_pos is None:
            return

        # Step trajectory time
        self.t = self.t + 1 / self.rate_hz

        # Trajectory time index cannot be higher than total time
        self.t = min(self.t, self.T)

        ''' ----- TODO: Compute interpolated pose at each time step --- '''

        # Position: linear interpolation
        alpha = self.t / self.T
        pos = (1 - alpha) * self.start_pos + alpha * self.goal_pos

        # Orientation: SLERP
        slerp = Slerp([0.0, 1.0], R.from_quat([self.start_quat, self.goal_quat]))
        rot = slerp(alpha)
        quat = rot.as_quat()

        ''' ----------------------------------------------------------- '''

        # Create and publish message
        msg = PoseStamped()

        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]

        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])

        msg.header.stamp = self.get_clock().now().to_msg()

        self.pub.publish(msg)

        if self.t == self.T:
            self.get_logger().info("Trajectory completed! Ctrl+C to exit")
            self.timer.destroy()
        else:
            self.get_logger().info(
                f"Publishing pose at t = {self.t:.2f} / {self.T:.2f} s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ExercisePosePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
