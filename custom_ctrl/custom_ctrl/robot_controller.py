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
import numpy as np
import copy
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from custom_ctrl.calibration import transformmsg_to_matrix
from custom_ctrl.calibration import matrix_to_transformmsg
from custom_ctrl_msgs.srv import MoveCommand
from custom_ctrl_msgs.msg import TaskStatus


class RobotController(Node):
    """
    Class that controls the robot depending on the current task and hand position. It contains all task functions.
    Structure:  init, callback functions, publish functions, task functions, helper functions
    """

    def __init__(self):
        super().__init__('robot_controller')
        self.group = ReentrantCallbackGroup()
        self.subscription_mode = self.create_subscription(
            String,
            'Unity/game_mode',
            self.mode_cb,
            1,
            callback_group=self.group
        )
        self.subscription_ee = self.create_subscription(
            Transform,
            'Robot/ee_pose',
            self.ee_cb,
            1,
            callback_group=self.group
        )
        self.subscription_hand_grab = self.create_subscription(
            Transform,
            'Control/hand_grab_pose',
            self.hand_grab_cb,
            1,
            callback_group=self.group
        )
        self.subscription_task_status = self.create_subscription(
            TaskStatus,
            'Control/task_status',
            self.task_status_cb,
            10,
            callback_group=self.group
        )
        self.publish_ee = self.create_publisher(
            Transform,
            '/Control/ee_pose_request',
            1)
        self.publish_req = self.create_publisher(
            Bool,
            '/Control/request_is_command',
            1)
        self.publish_gripper = self.create_publisher(
            String,
            '/Control/gripper',
            10)
        self.publish_task_status = self.create_publisher(
            TaskStatus,
            '/Control/task_status',
            10)
        self.move_request_cli = self.create_client(
            MoveCommand,
            'Control/ee_move_request',
            callback_group=self.group)
        while not self.move_request_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.ee_req = MoveCommand.Request()
        self.game_mode = "calibration"
        self.headpose = None
        self.hand_grab_matrix = None
        self.ee_pose_current = None
        self.ee_pose_requested = None
        self.ee_pose_commanded = None
        self.robot_range = 0.7  # meters
        self.pos_diff_max = 0.03  # meters
        self.rot_diff_max = 5  # degrees
        self.close_distance = 0.21  # meters a little bigger than bobble radius defined in safety_check.py
        self.hand_x_angle = 0  # degrees
        self.current_item = Items("cube")
        self.gripper_length = 0.06  # meters
        self.grab_pose = None
        self.bubble_grab_pose = None
        self.stop = False

    # Callback functions:

    def mode_cb(self, msg):
        """
        Callback function that stores the current game mode in 'self.game_mode'
        """
        if self.game_mode == msg.data:
            self.get_logger().info(f"Game mode is already set to: {self.game_mode}")
            return

        self.game_mode = msg.data
        self.get_logger().info(f"Game mode is set to: {self.game_mode}")

        if self.game_mode == "idle":
            self.stop = True
        else:
            self.stop = False

        if self.game_mode == "ready":
            self.move_to_ready_pose()

        # sleep and idle mode is handled in arm_kinematics.py

    def ee_cb(self, msg: Transform):
        """
        Callback function that stores the current pose of the robot end-effector
        """
        self.ee_pose_current = transformmsg_to_matrix(msg)

    def hand_grab_cb(self, msg):
        """
        Callback function that stores the current right hand pose
        """
        self.hand_grab_matrix = transformmsg_to_matrix(msg)
        self.calculate_hand_x_angle()

    def task_status_cb(self, msg: TaskStatus):
        """
        Callback function that triggers task functions
        """
        if msg.is_done:
            return

        if msg.item_name is not None:
            self.current_item = Items(msg.item_name)

        if msg.task_name == "grab_from_hand":
            self.grab_from_hand()
        elif msg.task_name == "place_in_hand":
            self.place_in_hand()
        elif msg.task_name == "place_in_storage":
            self.place_in_storage()
        elif msg.task_name == "grab_from_storage":
            self.grab_from_storage()
        elif msg.task_name == "look_at_hand":
            self.look_at_hand()
        elif msg.task_name == "go_to_ready":
            self.move_to_ready_pose()
        elif msg.task_name is None:
            pass
        else:
            self.get_logger().error("Task name is not valid")

        self.send_task_status(msg)
        return

    # publish functions:

    def send_ee_pose_request(self, ee_goal_pose, near_interaction=False):
        """
        function that sends the desired end-effector pose as a service request to the safety check
        :param ee_goal_pose: pose as 4x4 numpy array
        :param near_interaction: whether the robot is allowed to go into the hand bubble
        """
        send_msg = matrix_to_transformmsg(ee_goal_pose)
        self.publish_ee.publish(send_msg)

        self.ee_req.pose = send_msg
        self.ee_req.near_interaction = near_interaction
        future = self.move_request_cli.call_async(self.ee_req)

        self.wait_until_future_complete(future, timeout=0.4)
        if not future.done():
            self.get_logger().warn("Pose request timed out.")
            return False
        response = future.result()

        self.send_requested_is_commanded(response.is_valid)
        return response.is_valid

    def send_requested_is_commanded(self, is_true):
        """
        function that publishes whether the current motion request is executed or not.
        """
        msg = Bool()
        msg.data = is_true
        self.publish_req.publish(msg)

    def send_task_status(self, msg):
        """
        function that publishes completed tasks
        """
        msg.is_done = True
        self.publish_task_status.publish(msg)

    # task functions:

    def grab_from_hand(self):
        """
        function that defines the action of grabbing an object out of a human hand
        """
        self.get_logger().info(f"Grab {self.current_item.name} from hand!")

        self.wait(1)

        self.open_gripper()
        self.move_to_hand_bubble()
        self.wait(1)

        self.move_to_grab_pose()
        self.wait(1)

        self.close_gripper()
        self.wait(2)

        self.back_out_of_hand_bubble()
        self.wait(1)

        self.get_logger().info("Grab from hand completed.")
        return

    def place_in_hand(self):
        """
        function that defines the action of placing an object into a human hand. (reverse grab)
        """
        self.get_logger().info(f"Place {self.current_item.name} in hand!")

        self.wait(1)

        self.move_to_hand_bubble()
        self.wait(1)

        self.move_to_grab_pose()
        self.wait(1)

        self.open_gripper()
        self.wait(2)

        self.back_out_of_hand_bubble()
        self.wait(1)

        self.get_logger().info("Place in hand completed.")
        return

    def place_in_storage(self):
        """
        function that defines the action of placing an object into an object specific storage location.
        """
        self.get_logger().info(f"Place {self.current_item.name} in storage!")

        self.move_to_custom_pose(self.current_item.close_to_storage_pose)
        self.move_to_custom_pose(self.current_item.storage_pose)
        self.open_gripper()
        self.wait(2)
        self.move_to_custom_pose(self.current_item.close_to_storage_pose)
        self.wait(1)

        self.get_logger().info("Place in storage completed.")
        return

    def grab_from_storage(self):
        """
        function that defines the action of grabbing an object from its specific storage location.
        """
        self.get_logger().info(f"Grab {self.current_item.name} from storage!")

        self.open_gripper()
        self.move_to_custom_pose(self.current_item.close_to_storage_pose)
        self.move_to_custom_pose(self.current_item.storage_pose)
        self.close_gripper()
        self.wait(2)
        self.move_to_custom_pose(self.current_item.close_to_storage_pose)
        self.wait(1)

        self.get_logger().info("Grab from storage completed.")
        return

    def look_at_hand(self):
        """
        function that defines the action of placing itself between hand and base with a horizontal adaptive gripper.
        """
        hand_pose = copy.deepcopy(self.hand_grab_matrix)
        hand_position = hand_pose[:3, 3]
        goal_distance = self.robot_range * 0.6
        goal_position = normalize_vector(hand_position) * goal_distance
        goal_orientation = self.find_horizontal_adapted_parallel_grab_orientation()
        goal_pose = np.identity(4)
        goal_pose[:3, :3] = goal_orientation
        goal_pose[:3, 3] = goal_position

        self.send_ee_pose_request(goal_pose)
        self.wait(0.1)
        return

    # helper functions:

    def wait_until_future_complete(self, future, timeout=0.1):
        """
        function that waits for a service to complete
        :param future: Future event of service call
        :param timeout: Break the loop after this amount of seconds
        """
        t = time.time()
        while (not future.done()) and time.time() < t + timeout:
            continue

    def wait(self, seconds: float = 1):
        """
        wait function that considers stop condition
        """
        if not self.stop:
            time.sleep(seconds)

    def move_to_grab_pose(self):
        """
        loop that sends pose request until a solution is found
        :return: whether robot has arrived
        """
        while not self.stop:
            self.update_grab_pose_near_interaction()
            if self.send_ee_pose_request(self.grab_pose, near_interaction=True):
                break
            else:
                time.sleep(0.5)

        has_arrived = self.wait_until_robot_arrives_at(self.grab_pose)
        self.get_logger().info(f"Robot has moved to grab pose: {has_arrived}")
        return has_arrived

    def update_grab_pose_near_interaction(self):
        """
        function that defines how to approach the hand within the hand bubble based on the object
        """
        near_interaction_mode = copy.deepcopy(self.current_item.near_interaction_mode)
        bubble_grab_pose = copy.deepcopy(self.bubble_grab_pose)

        if near_interaction_mode == "static":
            pass
        elif near_interaction_mode == "adaptive":
            self.find_grab_pose()
        elif near_interaction_mode == "position_adaptive":
            self.find_grab_pose()
            self.grab_pose[:3, :3] = bubble_grab_pose[:3, :3]
        else:
            self.get_logger().error("The value of self.current_item.near_interaction_mode is not valid.")
            return False
        return True

    def move_to_hand_bubble(self):
        """
        loop that sends the motion request to move to the hand bubble until a solution is found.
        :return: whether robot has arrived
        """
        self.find_bubble_grab_pose()

        while not self.send_ee_pose_request(self.bubble_grab_pose) and not self.stop:
            time.sleep(0.5)
            self.find_bubble_grab_pose()
        has_arrived = self.wait_until_robot_arrives_at(self.bubble_grab_pose)
        self.get_logger().info(f"Robot has moved to hand bubble: {has_arrived}")
        return has_arrived

    def find_grab_pose(self):
        """
        function that finds the grab pose based on the orientation_mode, hand_grab_point and item
        :return: bool whether successful
        """
        orientation_mode = copy.deepcopy(self.current_item.orientation_mode)
        hand_grab_pose = copy.deepcopy(self.hand_grab_matrix)
        hand_grab_pos = hand_grab_pose[:3, 3]

        if orientation_mode == "horizontal_basic":
            grab_orientation = self.find_horizontal_grab_orientation()
        elif orientation_mode == "horizontal_turned":
            grab_orientation = self.find_horizontal_turned_grab_orientation()
        elif orientation_mode == "horizontal_adapted":
            grab_orientation = self.find_horizontal_adapted_grab_orientation()
        elif orientation_mode == "horizontal_adapted_parallel":
            grab_orientation = self.find_horizontal_adapted_parallel_grab_orientation()
        elif orientation_mode == "fully_adapted":
            grab_orientation = self.find_fully_adapted_grab_orientation()
        elif orientation_mode == "forward":
            grab_orientation = np.identity(3)
        else:
            self.get_logger().error("The value of 'self.current_item.orientation_mode' is not valid.")
            return False

        grab_pose = np.identity(4)
        grab_pose[:3, :3] = grab_orientation
        grab_pose[:3, 3] = hand_grab_pos

        grab_pose = shift_pose_by(self.gripper_length/2, grab_pose, backwards=True)

        grab_pose = grab_pose @ self.current_item.grab_pose_shift

        self.grab_pose = grab_pose
        return True

    def find_bubble_grab_pose(self):
        """
        function that calculates the pose located at the border of the hand bubble in an object specific orientation
        """
        self.find_grab_pose()
        grab_pose = copy.deepcopy(self.grab_pose)
        self.bubble_grab_pose = shift_pose_by(self.close_distance, grab_pose, backwards=True)
        return True

    def find_horizontal_grab_orientation(self):
        """
        function that calculates the orientation of the robot end-effector,
        so that the gripper is horizontal pointing from base to hand
        :return: orientation, 3x3 numpy array
        """
        hand_grab_pose = copy.deepcopy(self.hand_grab_matrix)
        hand_grab_pos = hand_grab_pose[:3, 3]
        x_rot = [hand_grab_pos[0], hand_grab_pos[1], 0]
        x_rot = x_rot / np.linalg.norm(x_rot)
        z_rot = [0, 0, 1]  # pointing up
        y_rot = np.cross(z_rot, x_rot)
        horizontal_grab_orientation = np.identity(3)
        horizontal_grab_orientation[:3, 0] = x_rot
        horizontal_grab_orientation[:3, 1] = y_rot
        horizontal_grab_orientation[:3, 2] = z_rot

        return horizontal_grab_orientation

    def find_horizontal_turned_grab_orientation(self):
        """
        function that calculates the orientation of the robot end-effector,
        so that the gripper is in the horizontal plane but turned 90 degrees pointing from base to hand
        :return: orientation, 3x3 numpy array
        """
        horizontal_grab_orientation = self.find_horizontal_grab_orientation()
        turn_matrix = np.array([[1, 0, 0],
                                [0, 0, 1],
                                [0, -1, 0]])
        grab_orientation = horizontal_grab_orientation @ turn_matrix
        return grab_orientation

    def find_horizontal_adapted_grab_orientation(self):
        """
        function that calculates the orientation of the robot end-effector,
        so that the gripper is in the horizontal plane but turned 90 degrees plus the hand rotation in ee-forward
        direction pointing from base to hand
        :return: orientation, 3x3 numpy array
        """
        horizontal_grab_orientation = self.find_horizontal_grab_orientation()
        x_rotation = R.from_euler(seq='XYZ', angles=[-self.hand_x_angle - 90, 0, 0], degrees=True)
        rot_around_x = x_rotation.as_matrix()
        grab_orientation = horizontal_grab_orientation @ rot_around_x
        return grab_orientation

    def find_horizontal_adapted_parallel_grab_orientation(self):
        """
        function that calculates the orientation of the robot end-effector,
        so that the gripper is in the horizontal plane but turned by the hand rotation in ee-forward
        direction pointing from base to hand
        :return: orientation, 3x3 numpy array
        """
        horizontal_grab_orientation = self.find_horizontal_grab_orientation()
        x_rotation = R.from_euler(seq='XYZ', angles=[-self.hand_x_angle, 0, 0], degrees=True)
        rot_around_x = x_rotation.as_matrix()
        grab_orientation = horizontal_grab_orientation @ rot_around_x
        return grab_orientation

    def calculate_hand_x_angle(self):
        """
        function that calculates the angle around the x-axis (roll) of the hand pose
        """
        hand_grab_orientation = self.hand_grab_matrix[:3, :3]
        R_0_ha = R.from_matrix(hand_grab_orientation)
        euler_angles = R_0_ha.as_euler(seq='XYZ', degrees=True)
        self.hand_x_angle = euler_angles[0]
        return euler_angles[0]

    def find_fully_adapted_grab_orientation(self):
        """
        function that calculates the orientation of the robot end-effector,
        so that the gripper is mirrored to the hand and turned by 90 degrees.
        :return: orientation, 3x3 numpy array
        """
        hand_grab_pose = copy.deepcopy(self.hand_grab_matrix)
        hand_grab_orientation = hand_grab_pose[:3, :3]
        hand_to_robot = np.array([[-1, 0, 0],
                                  [0, 0, 1],
                                  [0, 1, 0]])
        grab_orientation = hand_grab_orientation @ hand_to_robot
        return grab_orientation

    def back_out_of_hand_bubble(self):
        """
        function that sends a motion request so the robot leaves the hand bubble safely
        :return: whether robot arrived
        """
        # move to bubble_grab_pose is more consistent than moving back along x. Alternative:
        # actual_pose = copy.deepcopy(self.ee_pose_current)
        # goal_pose = shift_pose_by(self.close_distance, actual_pose, backwards=True)
        goal_pose = copy.deepcopy(self.bubble_grab_pose)

        self.send_ee_pose_request(goal_pose)

        has_arrived = self.wait_until_far_away()
        return has_arrived

    def wait_until_far_away(self, timeout_seconds=5):
        """
        function that waits until the robot has left the hand bubble
        :param timeout_seconds: function returns after this amount of seconds
        :return: whether successful
        """
        end_time = time.time() + timeout_seconds

        while not self.stop and time.time() < end_time:
            if not self.is_close_to_hand():
                return True
            else:
                time.sleep(0.1)

        return False

    def hand_robot_distance(self):
        """
        function that calculates the distance between the right hand pose and the robot end effector.
        :return: float distance
        """
        hand_pos = self.hand_grab_matrix[:3, 3]
        robot_pos = self.ee_pose_current[:3, 3]
        pos_diff = calculate_position_difference(hand_pos, robot_pos)
        return pos_diff

    def is_close_to_hand(self):
        """
        function that checks whether the robot end-effector is inside the hand bubble
        :return: bool
        """
        if self.hand_robot_distance() > self.close_distance:
            return False
        else:
            return True

    # general control functions

    def close_gripper(self):
        """
        send close gripper command to arm_kinematics
        """
        msg_to_send = String()
        msg_to_send.data = "grasp"
        if not self.stop:
            self.publish_gripper.publish(msg_to_send)

    def open_gripper(self):
        """
        send open gripper command to arm_kinematics
        """
        msg_to_send = String()
        msg_to_send.data = "release"
        if not self.stop:
            self.publish_gripper.publish(msg_to_send)

    def move_to_custom_pose(self, goal_pose):
        """
        send a motion request of a given pose
        :param goal_pose: numpy array 4x4
        """
        self.send_ee_pose_request(goal_pose)
        self.wait_until_robot_arrives_at(goal_pose)
        return

    def move_to_ready_pose(self):
        """
        send a motion request to move to the robot specific ready pose
        """
        self.open_gripper()
        goal_pose = np.array([[1, 0, 0, 0.3],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0.2],
                              [0, 0, 0, 1]])
        self.send_ee_pose_request(goal_pose)
        self.wait_until_robot_arrives_at(goal_pose)

    def wait_until_robot_arrives_at(self, goal_pose, timeout_seconds=5):
        """
        function that waits until the robot is close to a given pose
        :param goal_pose: numpy array 4x4
        :param timeout_seconds: return after this amount of seconds
        :return: whether robot arrived
        """
        end_time = time.time() + timeout_seconds

        while not self.stop and time.time() < end_time:
            if self.check_completed_movement(goal_pose):
                return True
            else:
                time.sleep(0.1)

        return False

    def check_completed_movement(self, commanded_pose):
        """
        function that checks if the robot is close to a given pose
        :param commanded_pose: numpy array 4x4
        :return: bool
        """
        actual_pose = copy.deepcopy(self.ee_pose_current)
        actual_pos = actual_pose[:3, 3]
        commanded_pos = commanded_pose[:3, 3]
        actual_rot = actual_pose[:3, :3]
        commanded_rot = commanded_pose[:3, :3]
        pos_diff = calculate_position_difference(actual_pos, commanded_pos)
        rot_diff = calculate_rotation_difference(actual_rot, commanded_rot)
        if pos_diff < self.pos_diff_max and rot_diff < self.rot_diff_max:
            return True
        else:
            return False


class Items:
    """
    Class that contains item information. Any item can be added
    """

    def __init__(self, name: str):
        self.name = name

        if name == "cube":
            self.orientation_mode = "horizontal_adapted"
            self.near_interaction_mode = "adaptive"
            self.grab_pose_shift = np.array([[1, 0, 0, 0],
                                             [0, 1, 0, 0],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])

            self.storage_pose = np.array([[0, 1, 0, 0],
                                          [0, 0, -1, -0.2],
                                          [-1, 0, 0, 0.05],
                                          [0, 0, 0, 1]])

        if name == "cup":
            self.orientation_mode = "horizontal_basic"
            self.near_interaction_mode = "adaptive"
            self.grab_pose_shift = np.array([[1, 0, 0, -0.02],
                                             [0, 1, 0, 0],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])
            s = np.sqrt(0.5)
            self.storage_pose = np.array([[0, s, s, 0.18],
                                          [0, s, -s, -0.20],
                                          [-1, 0, 0, 0.10],
                                          [0, 0, 0, 1]])

        if name == "pointer":
            self.orientation_mode = "horizontal_adapted_parallel"
            self.near_interaction_mode = "adaptive"
            self.grab_pose_shift = None
            self.storage_pose = None

        if name == "can":
            self.orientation_mode = "horizontal_adapted_parallel"
            self.near_interaction_mode = "adaptive"
            self.grab_pose_shift = np.array([[1, 0, 0, 0],
                                             [0, 1, 0, 0],
                                             [0, 0, 1, 0.05],
                                             [0, 0, 0, 1]])
            self.storage_pose = np.array([[0, 1, 0, 0],
                                          [0, 0, -1, -0.2],
                                          [-1, 0, 0, 0.05],
                                          [0, 0, 0, 1]])

        if name == "screwdriver_small":
            self.orientation_mode = "horizontal_adapted_parallel"
            self.near_interaction_mode = "adaptive"
            self.grab_pose_shift = np.array([[1, 0, 0, 0.01],
                                             [0, 1, 0, 0],
                                             [0, 0, 1, 0.03],
                                             [0, 0, 0, 1]])

            self.storage_pose = np.array([[0, 0, 1, 0.29],
                                          [0, 1, 0, 0.00],
                                          [-1, 0, 0, 0.05],
                                          [0, 0, 0, 1]])

        if name == "screwdriver_big":
            self.orientation_mode = "horizontal_basic"
            self.near_interaction_mode = "adaptive"
            self.grab_pose_shift = np.array([[1, 0, 0, 0.01],
                                             [0, 1, 0, 0],
                                             [0, 0, 1, 0.05],
                                             [0, 0, 0, 1]])

            self.storage_pose = np.array([[0, 0, 1, 0.29],
                                          [0, 1, 0, 0.10],
                                          [-1, 0, 0, 0.05],
                                          [0, 0, 0, 1]])

        if self.storage_pose is not None:
            self.close_to_storage_pose = shift_pose_by(0.1, self.storage_pose, backwards=True)
        self.workplace_pose = None

        # self.orientation_mode = "horizontal_adapted"
        # self.orientation_mode = "horizontal_adapted_parallel"
        # self.orientation_mode = "horizontal_basic"
        # self.orientation_mode = "horizontal_turned"
        # self.orientation_mode = "fully_adapted"
        # self.orientation_mode = "forward"
        # self.near_interaction_mode = "static"
        # self.near_interaction_mode = "adaptive"
        # self.near_interaction_mode = "position_adaptive"


# static functions:
def calculate_position_difference(pos1, pos2):
    pos_diff_vec = pos1 - pos2
    pos_diff_abs = np.linalg.norm(pos_diff_vec)
    return pos_diff_abs


def calculate_rotation_difference(rot1, rot2):
    rot2_inv = np.linalg.inv(rot2)
    rot_diff_mat = rot2_inv @ rot1
    rot_diff_abs = np.rad2deg(np.arccos((np.trace(rot_diff_mat) - 1) / 2))
    return rot_diff_abs


def normalize_vector(vector):
    vector_length = np.linalg.norm(vector)
    if vector_length == 0:
        return vector
    normalized_vector = vector / vector_length
    return normalized_vector


def shift_pose_by(length: float,  pose_matrix, backwards=False):
    shift = np.identity(4)
    if backwards:
        shift[0, 3] = -1 * length  # m
    else:
        shift[0, 3] = length  # m
    shifted_pose = pose_matrix @ shift
    return shifted_pose


def main(args=None):
    """ Spin the robot_controller node."""

    rclpy.init(args=args)
    try:
        robot_controller = RobotController()
        # MultiThreadedExecutor executes callbacks with a thread pool. If num_threads is not
        # specified then num_threads will be multiprocessing.cpu_count() if it is implemented.
        # Otherwise, it will use a single thread. This executor will allow callbacks to happen in
        # parallel, however the MutuallyExclusiveCallbackGroup in DoubleTalker will only allow its
        # callbacks to be executed one at a time. The callbacks in Listener are free to execute in
        # parallel to the ones in DoubleTalker ,however.
        executor = MultiThreadedExecutor(num_threads=10)
        executor.add_node(robot_controller)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            robot_controller.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
