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

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from geometry_msgs.msg import Transform
from custom_ctrl.calibration import transformmsg_to_matrix
from custom_ctrl.robot_controller import calculate_position_difference
from custom_ctrl_msgs.srv import MoveCommand
from std_msgs.msg import String


class SafetyCheck(Node):
    """
    Class that is responsible for the safety checks before a motion request is forwarded to the robot kinematics.
    """

    def __init__(self):
        super().__init__('safety_check')
        self.group = ReentrantCallbackGroup()
        self.subscription_hand_grab_pose = self.create_subscription(
            Transform,
            'Control/hand_grab_pose',
            self.hand_grab_pose_cb,
            10,
            callback_group=self.group)
        self.subscription_game_mode = self.create_subscription(
            String,
            'Unity/game_mode',
            self.game_mode_cb,
            10,
            callback_group=self.group)
        self.move_request_srv = self.create_service(
            MoveCommand,
            'Control/ee_move_request',
            self.ee_move_request_cb,
            callback_group=self.group
        )
        self.move_command_cli = self.create_client(
            MoveCommand,
            'Control/ee_move_command',
            callback_group=self.group
        )
        while not self.move_command_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.ee_req = MoveCommand.Request()
        self.hand_grab_pose = np.identity(4)
        self.safety_bubble_radius = 0.2  # m
        self.near_interaction = False
        self.current_game_mode = "calibration"

    def game_mode_cb(self, msg: String):
        """
        callback function that saves the current game mode
        """
        self.current_game_mode = msg.data

    def ee_move_request_cb(self, request, response):
        """
        callback function that gets motion requests, checks for safety and sends command to the robot
        :param request: MotionCommand service request containing pose and near_interaction value
        :param response: MotionCommand service response containing boolean is_valid
        :return: response: a MotionCommand service response containing boolean is_valid
        """
        pose_matrix = transformmsg_to_matrix(request.pose)
        near_interaction = request.near_interaction
        self.near_interaction = near_interaction

        if self.current_game_mode == "idle":
            self.get_logger().warn("In idle game mode no movement will be executed")
            response.is_valid = False
        elif self.request_in_static_collision(pose_matrix):
            self.get_logger().warn("Requested pose is in static collision.")
            response.is_valid = False
        elif not near_interaction and self.request_in_hand_collision(pose_matrix):
            self.get_logger().warn("Requested non near interaction pose is in hand collision.")
            response.is_valid = False
        else:
            response.is_valid = self.send_ee_pose_command(request)
        return response

    def request_in_static_collision(self, requested_pose):
        """
        function that checks whether there is a static collision with the requested ee pose
        :param requested_pose: 4x4 transformation matrix
        :return: True if in collision, False else
        """
        # check floor
        requested_z_pos = requested_pose[2, 3]
        if requested_z_pos < 0.02:
            return True

        # here other static objects could be added (e.g. wall)

        return False

    def request_in_hand_collision(self, requested_pose):
        """
        function that checks whether there is a hand bubble collision with the requested ee pose
        :param requested_pose: 4x4 transformation matrix
        :return: True if in collision, False else
        """
        requested_pos = requested_pose[:3, 3]
        hand_pos = self.hand_grab_pose[:3, 3]
        dist = calculate_position_difference(requested_pos, hand_pos)
        if dist < self.safety_bubble_radius:
            return True
        else:
            return False

    def send_ee_pose_command(self, request):
        """
        function that sends the ee pose command service to the robot arm kinematics
        :param request: MotionCommand service request containing pose and near_interaction value
        :return: bool whether a path was found
        """
        future_of_command = self.move_command_cli.call_async(request)
        self.wait_until_future_complete(future_of_command, timeout=0.25)
        if not future_of_command.done():
            self.get_logger().info("The ee pose command timed out.")
            return False
        response = future_of_command.result()
        return response.is_valid

    def wait_until_future_complete(self, future, timeout=0.1):
        """
        function that waits until the service response or the timeout has arrived
        :param future: future of service call
        :param timeout: time in seconds
        """
        t = time.time()
        while (not future.done()) and time.time() < t + timeout:
            continue

    def hand_grab_pose_cb(self, msg: Transform):
        """
        function that saves the current hand_grab_pose
        :param msg: Transform message
        """
        self.hand_grab_pose = transformmsg_to_matrix(msg)


def main(args=None):
    """ Spin the safety_check node."""

    rclpy.init(args=args)
    try:
        safety_check = SafetyCheck()
        # MultiThreadedExecutor executes callbacks with a thread pool. If num_threads is not
        # specified then num_threads will be multiprocessing.cpu_count() if it is implemented.
        # Otherwise, it will use a single thread. This executor will allow callbacks to happen in
        # parallel, however the MutuallyExclusiveCallbackGroup in DoubleTalker will only allow its
        # callbacks to be executed one at a time. The callbacks in Listener are free to execute in
        # parallel to the ones in DoubleTalker, however.
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(safety_check)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            safety_check.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
