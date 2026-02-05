# Copyright (c) 2026 Fabio Amadio
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformListener, Buffer, LookupException, TimeoutException
import time

class EndEffectorMarkerNode(Node):
    def __init__(self):
        super().__init__('ee_interactive_marker_node')

        # Declare parameters and get their values
        self.declare_parameter('des_pose_topic_name', '/cartesian_impedance/desired_pose')
        self.declare_parameter('base_link', 'base')
        self.declare_parameter('ee_link', 'fr3_hand_tcp')
        self.declare_parameter('curr_pose_topic_name', '/cartesian_impedance/current_pose')

        self.des_pose_topic_name = self.get_parameter('des_pose_topic_name').get_parameter_value().string_value
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.ee_link = self.get_parameter('ee_link').get_parameter_value().string_value
        self.curr_pose_topic_name = self.get_parameter('curr_pose_topic_name').get_parameter_value().string_value

        self.get_logger().info(f"Publishing to topic: {self.des_pose_topic_name}")
        self.get_logger().info(f"Base link: {self.base_link}, EE link: {self.ee_link}")

        # Callback group for concurrent handling
        self.callback_group = ReentrantCallbackGroup()

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped, self.des_pose_topic_name, 1, callback_group=self.callback_group
        )

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        time.sleep(1)

        # Interactive marker server
        self.server = InteractiveMarkerServer(self, 'ee_marker_server')

        self.enabled = False
        self.initialized = False

        self.menu = MenuHandler()
        self.reinit_entry = self.menu.insert("Reset", callback=self.handle_menu_feedback)
        self.checkbox_handle = self.menu.insert("Send goals", callback=self.on_menu_toggle)
        self.menu.setCheckState(self.checkbox_handle, MenuHandler.UNCHECKED)

        self.curr_pose_sub = self.create_subscription(
            PoseStamped, self.curr_pose_topic_name, self.curr_pose_cb, 1,
        )

    def curr_pose_cb(self, msg):
        if not self.initialized:
            self.get_logger().info('Initializing')
            self.initialized = self.try_initialize_marker()
                       
    def try_initialize_marker(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link, self.ee_link, rclpy.time.Time()
            )
            self.get_logger().info('TF transform acquired, initializing marker')
            self.create_interactive_marker(tf)
            return True
        except (LookupException, TimeoutException):
            self.get_logger().warn(f'Waiting for TF from {self.base_link} to {self.ee_link}...')
            time.sleep(1)
            return False

    def try_reinitialize_marker(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link, self.ee_link, rclpy.time.Time()
            )
            self.server.clear()
            self.create_interactive_marker(tf)
            self.get_logger().info("Marker reinitialized.")
        except (LookupException, TimeoutException):
            self.get_logger().warn("Could not reinitialize marker: TF not available.")

    def create_interactive_marker(self, tf):
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = self.base_link
        self.int_marker.name = 'ee_target'
        self.int_marker.description = 'Equilibrium Pose'
        self.int_marker.scale = 0.2

        # Set position and orientation
        self.int_marker.pose.position.x = tf.transform.translation.x
        self.int_marker.pose.position.y = tf.transform.translation.y
        self.int_marker.pose.position.z = tf.transform.translation.z
        self.int_marker.pose.orientation = tf.transform.rotation

        self.add_6dof_controls(self.int_marker)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"
        self.int_marker.controls.append(menu_control)

        self.server.insert(self.int_marker, feedback_callback=self.process_feedback)
        self.menu.apply(self.server, self.int_marker.name)
        self.server.applyChanges()

    def add_6dof_controls(self, marker):
        control_axes = [
            ('move_x', InteractiveMarkerControl.MOVE_AXIS, 1.0, 0.0, 0.0),
            ('move_y', InteractiveMarkerControl.MOVE_AXIS, 0.0, 1.0, 0.0),
            ('move_z', InteractiveMarkerControl.MOVE_AXIS, 0.0, 0.0, 1.0),
            ('rotate_x', InteractiveMarkerControl.ROTATE_AXIS, 1.0, 0.0, 0.0),
            ('rotate_y', InteractiveMarkerControl.ROTATE_AXIS, 0.0, 1.0, 0.0),
            ('rotate_z', InteractiveMarkerControl.ROTATE_AXIS, 0.0, 0.0, 1.0),
        ]

        for name, interaction_mode, x, y, z in control_axes:
            control = InteractiveMarkerControl()
            control.name = name
            control.interaction_mode = interaction_mode
            control.orientation.w = 1.0
            control.orientation.x = x
            control.orientation.y = y
            control.orientation.z = z
            marker.controls.append(control)

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE and self.enabled:
            pose_stamped = PoseStamped()
            pose_stamped.header = feedback.header
            pose_stamped.pose = feedback.pose
            self.pose_pub.publish(pose_stamped)
            # self.get_logger().info(f"Published EE reference pose: {pose_stamped.pose}")

    def handle_menu_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.menu_entry_id == self.reinit_entry:
            # self.get_logger().info("Reinitializing marker at current EE pose")
            self.try_reinitialize_marker()
    
    def on_menu_toggle(self, feedback):
        # Toggle checkbox state
        state = self.menu.getCheckState(self.checkbox_handle)
        new_state = MenuHandler.CHECKED if state != MenuHandler.CHECKED else MenuHandler.UNCHECKED
        self.menu.setCheckState(self.checkbox_handle, new_state)

        self.enabled = (new_state == MenuHandler.CHECKED)

        # Re-apply so RViz sees the updated checkmark
        self.menu.apply(self.server, self.int_marker.name)
        self.server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorMarkerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
