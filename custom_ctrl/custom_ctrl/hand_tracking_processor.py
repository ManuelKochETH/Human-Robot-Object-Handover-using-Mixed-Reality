# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import copy
import time
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Transform
from std_msgs.msg import Float32
from std_msgs.msg import String
from custom_ctrl.calibration import transformmsg_to_matrix
from custom_ctrl.calibration import matrix_to_transformmsg
from custom_ctrl.robot_controller import normalize_vector
from custom_ctrl.robot_controller import calculate_position_difference


class HandProcessor(Node):
    """
    Class that interprets the tracked hand for semantic information
    """

    def __init__(self):
        super().__init__('hand_tracking_processor')
        self.subscription_indexpose = self.create_subscription(
            Transform,
            'Control/index_pose',
            self.index_pose_cb,
            10)
        self.subscription_thumb_pose = self.create_subscription(
            Transform,
            'Control/thumb_pose',
            self.thumb_pose_cb,
            10)
        self.subscription_proximal_pose = self.create_subscription(
            Transform,
            'Control/proximal_pose',
            self.proximal_pose_cb,
            10)
        self.subscription_gesture = self.create_subscription(
            String,
            'Unity/gesture',
            self.gesture_cb,
            10)
        self.publisher_hand_grab_pose = self.create_publisher(
            Transform,
            'Control/hand_grab_pose',
            10)
        self.publisher_hand_velocity = self.create_publisher(
            Float32,
            'Control/hand_velocity',
            10)
        self.publisher_signal = self.create_publisher(
            String,
            'Control/hand_signal',
            10)

        self.index_pose = np.identity(4)
        self.proximal_pose = np.identity(4)
        self.thumb_pose = np.identity(4)
        self.hand_grab_pose = np.identity(4)
        self.gesture = "none"
        self.signal = None
        self.signal_buffer = [None, None, None]

        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.hand_velocity = 0.0  # meter per seconds
        self.max_interaction_distance = 0.6  # meters
        self.hand_velocity_limit = 0.05  # meter per seconds

    def timer_callback(self):
        """
        constantly repeated function to publish the poses
        """
        previous_hand_grab_pose = copy.deepcopy(self.hand_grab_pose)
        if self.find_hand_grab_point():
            send_msg = matrix_to_transformmsg(self.hand_grab_pose)
            self.publisher_hand_grab_pose.publish(send_msg)

            self.find_hand_velocity(previous_hand_grab_pose)
            self.send_hand_velocity()

            self.check_for_right_hand_await_signal()

    def gesture_cb(self, msg: String):
        """
        callback function obtaining hand gestures from unity
        """
        self.gesture = msg.data
        self.check_and_send_signal()

    def check_for_right_hand_await_signal(self):
        """
        function that checks whether the right hand is within the robot reach and not moving to send awaiting signal
        """
        if self.hand_velocity_limit > self.hand_velocity > 0.0:
            right_hand_position = copy.deepcopy(self.hand_grab_pose[:3, 3])
            # if np.linalg.norm(right_hand_position) < self.max_interaction_distance:
            #     self.send_awaiting()
            if right_hand_position[1] < 0.35:
                self.send_awaiting()

    def check_and_send_signal(self):
        """
        function that fills the signal buffer and sends a signal as soon a gesture consistently is recognised
        """
        self.signal_buffer.pop(0)
        self.signal_buffer.append(self.gesture)
        if not self.signal_buffer[:-1] == self.signal_buffer[1:]:
            self.gesture = "none"
        if not self.signal == self.gesture:
            self.send_signal()

    def send_signal(self):
        """
        function that publishes the signal
        """
        msg = String()
        msg.data = self.gesture
        self.publisher_signal.publish(msg)
        self.signal = copy.deepcopy(self.gesture)

    def send_awaiting(self):
        """
        function that publishes awaiting signal
        """
        msg = String()
        msg.data = "awaiting"
        self.publisher_signal.publish(msg)
        time.sleep(0.2)
        self.send_signal()

    def find_hand_velocity(self, previous_pose):
        """
        function that calculates the right hand velocity and stores it in 'self.hand_velocity'
        :param previous_pose: numpy array 4x4
        """
        actual_position = self.hand_grab_pose[:3, 3]
        previous_position = previous_pose[:3, 3]
        dist = calculate_position_difference(actual_position, previous_position)
        self.hand_velocity = dist / self.timer_period

    def send_hand_velocity(self):
        """
        function that publishes the right hand velocity
        """
        msg = Float32()
        msg.data = self.hand_velocity
        self.publisher_hand_velocity.publish(msg)

    def find_hand_grab_point(self):
        """
        function that calculates the hand_grab_pose which is located between index tip and thumb tip of the right hand
        """
        index_pos = self.index_pose[:3, 3]
        thumb_pos = self.thumb_pose[:3, 3]
        proximal_pos = self.proximal_pose[:3, 3]
        hand_grab_pose = np.identity(4)

        hand_grab_pos = (thumb_pos + index_pos) * 0.5
        if np.count_nonzero(hand_grab_pos) == 0:
            return False
        v_it = thumb_pos - index_pos
        v_pi = index_pos - proximal_pos
        v_pt = thumb_pos - proximal_pos

        y_rot = normalize_vector(v_it)
        z_rot = np.cross(v_pi, v_pt)
        z_rot = normalize_vector(z_rot)
        x_rot = np.cross(y_rot, z_rot)

        hand_grab_pose[:3, 3] = hand_grab_pos
        hand_grab_pose[:3, 0] = x_rot
        hand_grab_pose[:3, 1] = y_rot
        hand_grab_pose[:3, 2] = z_rot
        self.hand_grab_pose = hand_grab_pose
        return True

    def index_pose_cb(self, msg):
        """
        Callback function that saves the pose of the right hand index tip
        """
        self.index_pose = transformmsg_to_matrix(msg)

    def thumb_pose_cb(self, msg):
        """
        Callback function that saves the pose of the right hand thumb tip
        """
        self.thumb_pose = transformmsg_to_matrix(msg)

    def proximal_pose_cb(self, msg):
        """
        Callback function that saves the pose of the right proximal thumb joint
        """
        self.proximal_pose = transformmsg_to_matrix(msg)


def main(args=None):
    """ Spin the hand_tracking_processor node."""

    rclpy.init(args=args)
    hand_processor = HandProcessor()
    rclpy.spin(hand_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hand_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
