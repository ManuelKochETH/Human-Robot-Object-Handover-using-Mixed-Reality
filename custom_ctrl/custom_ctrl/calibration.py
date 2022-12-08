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
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import cv2  # OpenCV library
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import numpy as np
from scipy.spatial.transform import Rotation as R


class Calibration(Node):
    """
    Class that calibrates the position of the real robot in Unity with OpenCV
    Naming convention: T_ur is a Transformation matrix transforming a vector to another coordinate system according to
    v_u = T_ur * v_r with u for unity and r for robot. T_ur is therefore the transformation from the unity origin to the
    real robot origin with a right-handed z-up x-forward orientation. On the other hand uT_ur is the same Transformation
    but to the robot origin in unity with a unity conform left-handed y-up z-forward orientation. Additionally, c stands
    for the HoloLens2 RGB camera and a6, a8, a9 for the aruco marker and its index.
    """

    def __init__(self):
        super().__init__('calibration')
        self.subscription_image = self.create_subscription(
            Image,
            '/Camera/Image',
            self.image_cb,
            10)
        self.subscription_camera = self.create_subscription(
            Transform,
            '/Camera/uT_uc',
            self.camera_transformation_cb,
            10)
        self.publisher_aruco = self.create_publisher(
            Transform,
            '/Unity/uT_ua',
            10)
        self.publisher_robot = self.create_publisher(
            Transform,
            '/Unity/uT_ur',
            10)

        self.br = CvBridge()
        self.uT_uc = np.identity(4)
        self.uT_uc_updated = False
        self.T_ca_updated = False
        self.T_ca6 = np.identity(4)
        self.T_ca8 = np.identity(4)
        self.T_ca9 = np.identity(4)
        self.marker_size = 0.05  # 0.05 meter
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        # dictionary storing marker ids of robot base
        self.markersRobot = {6: False, 8: False, 9: False}
        self.show_image = False

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        """
        Callback function that is called repeatedly after the timer_period.
        It checks update parameters and triggers the calibration publications.
        """
        if self.T_ca_updated and self.uT_uc_updated:

            # Transformation from robot to unity coordinates
            robot_to_unity = np.zeros((4, 4))
            robot_to_unity[0, 2] = 1
            robot_to_unity[1, 0] = -1
            robot_to_unity[2, 1] = 1
            robot_to_unity[3, 3] = 1

            # Transformation from unity to OpenCV coordinates (right-handed as robot)
            flipaxis_y = np.identity(4)
            flipaxis_y[1, 1] = -1  # identity matrix with second element negative

            # calculate aruco transformations in unity frame
            uT_ua6 = self.uT_uc @ flipaxis_y @ self.T_ca6 @ robot_to_unity
            uT_ua8 = self.uT_uc @ flipaxis_y @ self.T_ca8 @ robot_to_unity
            uT_ua9 = self.uT_uc @ flipaxis_y @ self.T_ca9 @ robot_to_unity
            self.publish_marker_positions(uT_ua6)
            self.publish_marker_positions(uT_ua8)
            self.publish_marker_positions(uT_ua9)
            self.publish_robot_position(uT_ua6, uT_ua8, uT_ua9)

            self.T_ca_updated = False
            self.uT_uc_updated = False

            self.get_logger().info('Published: uT_ur and T_ca')
            self.get_logger().info('Calibration complete.')

        self.i += 1

    def publish_robot_position(self, T1, T2, T3):
        """
        Function that calculates and publishes the robot Transformation matrix
        :param T1: 4x4 Transformation matrix of first aruco marker
        :param T2: 4x4 Transformation matrix of second aruco marker
        :param T3: 4x4 Transformation matrix of third aruco marker
        """

        # calculate average marker position with rotation of first marker
        matrix = np.identity(4)
        matrix[:3, :3] = T1[:3, :3]
        t1 = T1[:3, 3]
        t2 = T2[:3, 3]
        t3 = T3[:3, 3]
        matrix[0, 3] = (t1[0] + t2[0] + t3[0])/3
        matrix[1, 3] = (t1[1] + t2[1] + t3[1])/3
        matrix[2, 3] = (t1[2] + t2[2] + t3[2])/3

        # offset of average marker position to robot base in unity coordinate system
        offset = np.identity(4)
        offset[2, 3] = -0.037  # offset in z-direction in [m]

        uT_ur = matrix @ offset
        msg = matrix_to_transformmsg(uT_ur)
        self.publisher_robot.publish(msg)

    def publish_marker_positions(self, matrix):
        """
        Function that publishes aruco marker positions
        :param matrix: 4x4 transformation matrix
        """
        msg = matrix_to_transformmsg(matrix)
        self.publisher_aruco.publish(msg)

    def camera_transformation_cb(self, msg):
        """
        callback function that saves the camera transformation matrix uT_uc
        :param msg: Transform ROS message containing the camera transform in unity frame
        """
        self.get_logger().info('uT_uc received.')
        matrix = transformmsg_to_matrix(msg)
        self.uT_uc = matrix
        self.uT_uc_updated = True

    def image_cb(self, msg):
        """
        callback function that triggers the aruco marker detection
        :param msg: Image ROS message
        """
        self.get_logger().info('Image received!')

        # Convert ROS Image message to OpenCV image
        img_cv2 = self.br.imgmsg_to_cv2(msg, "rgba8")
        img_cv2 = cv2.flip(img_cv2, 0)

        # Aruco calculation
        self.aruco_calculation(image=img_cv2, show_image=self.show_image)

    def aruco_calculation(self, image, show_image):
        """
        Function that detects aruco markers on an image and calculates their pose given the camera intrinsics.
        :param image: OpenCV image in grayscale
        :param show_image: Boolean whether to show an image with the detected markers
        """
        # Load camera intrinsics
        matrix_coefficients, distortion_coefficients = load_calib()

        aruco_params = cv2.aruco.DetectorParameters_create()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=aruco_params,
                                                                    cameraMatrix=matrix_coefficients,
                                                                    distCoeff=distortion_coefficients)
        # If markers are detected
        if len(corners) > 0:

            ids = np.array(ids).flatten()
            self.get_logger().info('Ids of detected Aruco markers: %s' % ids)

            # iterate through detected markers and save pose if marker id corresponds to dictionary
            for i in range(0, len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_size,
                                                                               matrix_coefficients,
                                                                               distortion_coefficients)
                # Draw Axis on visualization
                cv2.aruco.drawAxis(image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.02)

                if ids[i] == 6:
                    self.T_ca6 = get_aruco_matrix(rvec, tvec)
                    self.markersRobot[6] = True
                if ids[i] == 8:
                    self.T_ca8 = get_aruco_matrix(rvec, tvec)
                    self.markersRobot[8] = True
                if ids[i] == 9:
                    self.T_ca9 = get_aruco_matrix(rvec, tvec)
                    self.markersRobot[9] = True

            if show_image:
                cv2.imshow('Estimated Pose', image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            if list(self.markersRobot.values()) == [True, True, True]:
                self.T_ca_updated = True
                self.markersRobot = {6: False, 8: False, 9: False}
                self.get_logger().info('Aruco marker detection successful.')
            else:
                self.get_logger().error('Detection failed. Not all the necessary markers found. Please retake the '
                                        'picture.')

        else:
            self.get_logger().error('Detection failed. No markers found. Please retake the picture.')


def get_aruco_matrix(rvec, tvec):
    """
    Function that calculates a 4x4 matrix from rotation and translation vector
    :param rvec: rotation vector
    :param tvec: translation vector
    :return:  4x4 numpy matrix
    """
    r = R.from_rotvec(rvec[0][0])
    matrix = np.identity(4)
    matrix[:3, :3] = r.as_matrix()
    matrix[:3, 3] = tvec[0][0]
    return matrix


def load_calib():
    """
    function that loads the HoloLens2 RGB camera intrinsics and distortion obtained by camera calibration
    :return: mtx, dist: 2 camera matrices
    """
    mtx = np.array([[810.8497186,   0.0,            544.55264888],
                    [0.0,           822.25444378,   340.18380582],
                    [0.0,           0.0,            1.0]])
    dist = np.array([[-0.117178981, 2.13382594, -0.0134998017, 0.00334875537, -8.7769414]])
    return mtx, dist


def transformmsg_to_matrix(msg):
    """
    Function that calculates a Transformation matrix from a Transform message.
    :param msg: Transform ROS message containing Vector3 and Quaternion
    :return: 4x4 Tranformation matrix
    """
    trans = msg.translation
    quat = msg.rotation
    r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    matrix = np.identity(4)
    matrix[:3, :3] = r.as_matrix()
    matrix[:3, 3] = np.array([trans.x, trans.y, trans.z])
    return matrix


def matrix_to_transformmsg(matrix):
    """
    Function that calculates a Transform message from a Transformation matrix.
    :param matrix: 4x4 Tranformation matrix
    :return: Transform ROS message containing Vector3 and Quaternion
    """
    trans = matrix[:3, 3]
    r = R.from_matrix(matrix[:3, :3])
    quat = r.as_quat()

    msg_trans = Vector3()
    msg_trans.x = trans[0]
    msg_trans.y = trans[1]
    msg_trans.z = trans[2]
    msg_quat = Quaternion()
    msg_quat.x = quat[0]
    msg_quat.y = quat[1]
    msg_quat.z = quat[2]
    msg_quat.w = quat[3]
    msg = Transform()
    msg.translation = msg_trans
    msg.rotation = msg_quat
    return msg


def main(args=None):
    """ Spin the calibration node."""

    rclpy.init(args=args)
    calib = Calibration()
    rclpy.spin(calib)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    calib.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
