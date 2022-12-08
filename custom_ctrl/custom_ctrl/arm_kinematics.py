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

import rclpy
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from rclpy.utilities import remove_ros_args
import argparse
import sys
from custom_ctrl.calibration import transformmsg_to_matrix
from custom_ctrl.calibration import matrix_to_transformmsg
from custom_ctrl_msgs.srv import MoveCommand
from std_msgs.msg import String
from geometry_msgs.msg import Transform


class ArmKinematics(InterbotixManipulatorXS):
    """
    Class that accesses the Interbotix python API when ROS messages are received.
    """

    current_loop_rate = 25
    default_moving_time = 1.5  # regular speed
    default_accel_time = 0.5
    slow_moving_time = 2  # slow speed for near_interaction
    slow_accel_time = 0.5

    def __init__(self, pargs, args=None):
        InterbotixManipulatorXS.__init__(
            self,
            robot_model=pargs.robot_model,
            robot_name=pargs.robot_name,
            moving_time=self.default_moving_time,  # 0.2
            accel_time=self.default_accel_time,  # 0.1
            start_on_init=True,
            gripper_pressure=0.5,
            args=args
        )
        self.core.declare_parameter('robot_model', '')
        self.rate = self.core.create_rate(self.current_loop_rate)

        self.core.create_subscription(
            String,
            'Unity/game_mode',
            self.mode_cb,
            10)
        self.core.create_subscription(
            String,
            'Control/gripper',
            self.gripper_cb,
            10)
        self.ee_publisher = self.core.create_publisher(
            Transform,
            'Robot/ee_pose',
            10)
        self.move_command_srv = self.core.create_service(
            MoveCommand,
            'Control/ee_move_command',
            self.ee_move_command_cb)

        self.core.get_logger().info('Ready to receive end-effector commands.')

    def publish_ee_pose(self):
        """
        function that publishes the current end-effector pose
        """
        T_sb = self.arm.get_ee_pose()
        send_msg = matrix_to_transformmsg(T_sb)
        self.ee_publisher.publish(send_msg)

    def ee_move_command_cb(self, request, response):
        """
        service request callback that gets a movement request and searches for a solution and executes
        :return: response whether solution was found
        """
        pose_matrix = transformmsg_to_matrix(request.pose)
        if request.near_interaction:
            solution_found = self.arm.set_ee_pose_matrix(pose_matrix,
                                                         moving_time=self.slow_moving_time,
                                                         accel_time=self.slow_accel_time,
                                                         blocking=False)
        else:
            solution_found = self.arm.set_ee_pose_matrix(pose_matrix,
                                                         moving_time=self.default_moving_time,
                                                         accel_time=self.default_accel_time,
                                                         blocking=False)
        response.is_valid = solution_found[1]
        return response

    def mode_cb(self, msg):
        """
        callback function that listens to the current game mode
        """
        if msg.data == "sleep":
            self.arm.go_to_sleep_pose(
                moving_time=self.slow_moving_time,
                accel_time=self.slow_accel_time,
                blocking=True
            )
            self.arm.set_trajectory_time(moving_time=self.default_moving_time, accel_time=self.default_accel_time)
        if msg.data == "idle":
            # if idle stay where you are (so running motions are stopped)
            pose_matrix = self.arm.get_ee_pose()
            self.arm.set_ee_pose_matrix(pose_matrix)

    def gripper_cb(self, msg):
        """
        Callback function that executes gripper commands
        """
        delay = 1  # time for action in seconds
        if msg.data == "grasp":
            self.gripper.grasp(delay=delay)
        if msg.data == "release":
            self.gripper.release(delay=delay)

    def controller(self):
        # constantly repeated function.
        self.publish_ee_pose()
        return

    def start_robot(self):
        """
        Start function that connects the node to the API
        """
        try:
            self.start()
            while rclpy.ok():
                self.controller()
                self.rate.sleep()
        except KeyboardInterrupt:
            self.shutdown()


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--robot_model', default="wx250s")
    p.add_argument('--robot_name', default=None)
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    bot = ArmKinematics(ros_args, args=args)
    bot.start_robot()


if __name__ == '__main__':
    main()
