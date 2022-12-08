from setuptools import setup
from glob import glob
import os

package_name = 'custom_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mankoch',
    maintainer_email='mankoch@ethz.ch',
    description='Robot control with messages obtained from unity',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'robot_controller = custom_ctrl.robot_controller:main',
                'unity_ros_transform = custom_ctrl.unity_ros_transform:main',
                'calibration = custom_ctrl.calibration:main',
                'arm_kinematics = custom_ctrl.arm_kinematics:main',
                'hand_tracking_processor = custom_ctrl.hand_tracking_processor:main',
                'safety_check = custom_ctrl.safety_check:main',
                'task_controller = custom_ctrl.task_controller:main',
        ],
    },

)
