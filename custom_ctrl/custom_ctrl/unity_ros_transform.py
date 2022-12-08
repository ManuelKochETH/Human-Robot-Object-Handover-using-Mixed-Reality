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
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Transform
from custom_ctrl.calibration import transformmsg_to_matrix
from custom_ctrl.calibration import matrix_to_transformmsg


class UnityRosTransform(Node):
    """
    Class that handles the transformation of poses coming from Unity to the robot coordinate
    system and vice versa.
    Naming convention: T_ur is a Transformation matrix transforming a vector to another coordinate system according to
    v_u = T_ur * v_r with u for unity-origin and r for robot-origin. uT_ur is the same Transformation but in a unity
    conform left-handed y-up z-forward orientation. Ros uses a right-handed z-up x-forward orientation.
    """

    lhs_to_rhs = np.array([[0, -1, 0, 0],
                           [0, 0, 1, 0],
                           [1, 0, 0, 0],
                           [0, 0, 0, 1]])

    def __init__(self):
        super().__init__('unity_ros_transform')
        self.subscription_index_pose = self.create_subscription(
            Transform,
            'Unity/index_pose',
            self.index_pose_cb,
            10)
        self.subscription_thumb_pose = self.create_subscription(
            Transform,
            'Unity/thumb_pose',
            self.thumb_pose_cb,
            10)
        self.subscription_proximal_pose = self.create_subscription(
            Transform,
            'Unity/proximal_pose',
            self.proximal_pose_cb,
            10)
        self.subscription_hand_grab = self.create_subscription(
            Transform,
            'Control/hand_grab_pose',
            self.hand_grab_cb,
            10)
        self.subscription_calib = self.create_subscription(
            Transform,
            'Unity/uT_ur_final',
            self.calibration_cb,
            10)
        self.subscription_ee = self.create_subscription(
            Transform,
            'Control/ee_pose_request',
            self.ee_pose_cb,
            10)
        self.publish_index_pose = self.create_publisher(
            Transform,
            'Control/index_pose',
            10)
        self.publish_thumb_pose = self.create_publisher(
            Transform,
            'Control/thumb_pose',
            10)
        self.publish_proximal_pose = self.create_publisher(
            Transform,
            'Control/proximal_pose',
            10)
        self.publish_hand_grab = self.create_publisher(
            Transform,
            'Unity/hand_grab_pose',
            10)
        self.publish_ee = self.create_publisher(
            Transform,
            'Unity/ee_pose_request',
            10)
        self.T_ur = np.identity(4)
        self.T_ru = np.identity(4)

    def calibration_cb(self, msg):
        """
        Callback function triggered by the calibration step.
        It stores the transformation matrix from unity-origin to ros-origin in ros frame.
        It is needed as a calibration step for the pose calculation.
        """
        uT_ur = transformmsg_to_matrix(msg)
        self.T_ur = uT_ur @ self.lhs_to_rhs
        self.T_ru = np.linalg.inv(self.T_ur)

    def index_pose_cb(self, msg):
        """
        Callback function that calculates and publishes the right hand index tip pose in robot coordinates.
        :param msg: The message received containing the index pose in Unity coordinates.
        """
        msg_send = self.unity_to_ros_transformation(msg)
        self.publish_index_pose.publish(msg_send)

    def thumb_pose_cb(self, msg):
        """
        Callback function that calculates and publishes the right hand thumb tip pose in robot coordinates.
        :param msg: The message received containing the thumb pose in Unity coordinates.
        """
        msg_send = self.unity_to_ros_transformation(msg)
        self.publish_thumb_pose.publish(msg_send)

    def proximal_pose_cb(self, msg):
        """
        Callback function that calculates and publishes the right hand proximal thumb joint pose in robot coordinates.
        :param msg: The message received containing the proximal pose in Unity coordinates.
        """
        msg_send = self.unity_to_ros_transformation(msg)
        self.publish_proximal_pose.publish(msg_send)

    def hand_grab_cb(self, msg):
        """
        Callback function that calculates and publishes the right hand grab pose in Unity coordinates.
        :param msg: The message received containing the hand grab pose in ros coordinates.
        """
        msg_send = self.ros_to_unity_transformation(msg)
        self.publish_hand_grab.publish(msg_send)

    def ee_pose_cb(self, msg):
        """
        Callback function that calculates and publishes the requested robot end-effector pose in Unity coordinates.
        :param msg: The message received containing the requested end-effector pose in ros coordinates.
        """
        msg_send = self.ros_to_unity_transformation(msg)
        self.publish_ee.publish(msg_send)

    def unity_to_ros_transformation(self, unity_msg):
        """
        function that transforms a pose from unity-origin lhs to ros-origin rhs
        :param unity_msg: TransformMsg containing unity pose
        :return: TransformMsg containing ros pose
        """
        unity_pose = transformmsg_to_matrix(unity_msg)
        ros_pose = self.T_ru @ unity_pose @ self.lhs_to_rhs
        ros_msg = matrix_to_transformmsg(ros_pose)
        return ros_msg

    def ros_to_unity_transformation(self, ros_msg):
        """
        function that transforms a pose from ros-origin rhs to unity-origin lhs
        :param ros_msg: TransformMsg containing ros pose
        :return: TransformMsg containing unity pose
        """
        ros_pose = transformmsg_to_matrix(ros_msg)
        unity_pose = self.T_ur @ ros_pose @ np.linalg.inv(self.lhs_to_rhs)
        unity_msg = matrix_to_transformmsg(unity_pose)
        return unity_msg


def main(args=None):
    """ Spin the unity_ros_transform node."""

    rclpy.init(args=args)
    unity_ros_transform = UnityRosTransform()
    rclpy.spin(unity_ros_transform)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    unity_ros_transform.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
