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
import copy
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from custom_ctrl_msgs.msg import TaskStatus
from std_msgs.msg import String


class TaskController(Node):
    """
    Class that handles the task sequences and sends the triggered current task to the robot controller
    """

    def __init__(self):
        super().__init__('task_controller')
        self.group = ReentrantCallbackGroup()
        self.subscription_game_mode = self.create_subscription(
            String,
            'Unity/game_mode',
            self.game_mode_cb,
            10,
            callback_group=self.group)
        self.subscription_hand_signal = self.create_subscription(
            String,
            'Control/hand_signal',
            self.hand_signal_cb,
            10,
            callback_group=self.group)
        self.subscription_task_status = self.create_subscription(
            TaskStatus,
            'Control/task_status',
            self.task_status_cb_topic,
            10,
            callback_group=self.group
        )
        self.publisher_task_status = self.create_publisher(
            TaskStatus,
            'Control/task_status',
            10)
        self.publisher_game_mode = self.create_publisher(
            String,
            'Unity/game_mode',
            10)
        self.stop = False

        self.hand_signal = "none"
        self.game_mode = "calibration"
        self.stop_signal = "five"
        self.has_to_be_repeated = False
        self.current_sequence = None
        self.current_task = None

        # example task sequences:

        self.task_sequence_1 = [Task("grab_from_storage", "cup", "thumbs_up", None),
                                Task("place_in_hand", "cup", None, None),
                                Task("grab_from_storage", "cube", None, None),
                                Task("place_in_hand", "cube", None, None)]

        self.task_sequence_2 = [Task("grab_from_hand", "cup", None, None),
                                Task("place_in_storage", "cup", None, None)]

        self.task_sequence_3 = [Task("look_at_hand", "pointer", "thumbs_up", "always")]

        self.task_sequence_4 = [Task("grab_from_hand", "can", "thumbs_up", None),
                                Task("place_in_storage", "can", None, None)]

        self.task_sequence_5 = [Task("grab_from_hand", "cube", "awaiting", "index_up"),
                                Task("place_in_storage", "cube", None, None),
                                Task("grab_from_storage", "cup", None, None),
                                Task("place_in_hand", "cup", "awaiting", None)]

        self.task_sequence_study = [Task("grab_from_hand", "cube", "awaiting", None),
                                    Task("place_in_storage", "cube", None, None),
                                    Task("grab_from_storage", "cube", "thumbs_up", None),
                                    Task("place_in_hand", "cube", "awaiting", None)]

        self.task_sequence_video = [Task("place_in_hand", "cube", "thumbs_up", None),
                                     Task("grab_from_storage", "screwdriver_small", None, None),
                                     Task("place_in_hand", "screwdriver_small", "awaiting", None),
                                     Task("go_to_ready", "pointer", None, None),
                                     Task("grab_from_hand", "screwdriver_small", "awaiting", "index_up"),
                                     Task("place_in_storage", "screwdriver_small", None, None),
                                     Task("grab_from_storage", "screwdriver_big", None, None),
                                     Task("place_in_hand", "screwdriver_big", "awaiting", None),
                                     Task("go_to_ready", "pointer", None, None),
                                     Task("grab_from_hand", "screwdriver_big", "awaiting", "index_up"),
                                     Task("place_in_storage", "screwdriver_big", None, None)]


        self.task_sequence_demo = [Task("grab_from_hand", "cube", "thumbs_up", "index_up"),
                                   Task("place_in_storage", "cube", None, None)]

        self.task_sequence_to_execute = self.task_sequence_demo

    # callback functions:

    def game_mode_cb(self, msg: String):
        """
        callback function that updates the current game mode
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

        if self.game_mode == "perform_task":
            self.start_task_sequence(self.task_sequence_to_execute)

    def hand_signal_cb(self, msg: String):
        """
        callback function that handles incoming hand signals
        """
        self.hand_signal = msg.data

        if self.hand_signal == self.stop_signal:
            self.set_game_mode_to("idle")
            self.get_logger().info("Signal to stop received.")

        if self.current_task is not None:
            if self.hand_signal == self.current_task.repeat_signal:
                self.has_to_be_repeated = True
                self.get_logger().info("Signal for task repetition received.")

    def task_status_cb_topic(self, msg: TaskStatus):
        """
        callback function that is triggered when a task is completed.
        It triggers the next task or repeats the current.
        """
        if not msg.is_done:
            return
        self.get_logger().info(f"{msg.task_name} is completed.")

        if self.has_to_be_repeated or self.current_task.repeat_signal == "always":
            self.has_to_be_repeated = False
            # self.run_task()
            self.send_task_command()
            self.get_logger().info("Start task repetition.")
        else:
            self.run_next_task()

    # publisher functions:

    def set_game_mode_to(self, mode_name: str):
        """
        function that publishes the given game mode
        """
        msg = String()
        msg.data = mode_name
        self.publisher_game_mode.publish(msg)

    def send_task_command(self):
        """
        function that publishes the current task to robot_controller
        """
        if not self.stop:
            msg = TaskStatus()
            msg.task_name = self.current_task.name
            msg.item_name = self.current_task.item
            msg.is_done = False
            self.publisher_task_status.publish(msg)

    # control functions

    def start_task_sequence(self, task_sequence):
        """
        function that starts the first task of a given task sequence
        """
        self.current_sequence = copy.deepcopy(task_sequence)
        self.get_logger().info("Start sequence.")

        self.update_current_task()
        self.run_task()

    def run_next_task(self):
        """
        function that removes the completed task and starts the next one
        """
        self.current_sequence.pop(0)
        self.update_current_task()
        self.run_task()

    def update_current_task(self):
        """
        function that selects the current task based on the current task sequence
        """
        remaining_tasks = len(self.current_sequence)
        if remaining_tasks == 0:
            self.current_task = None
            self.set_game_mode_to("ready")
            self.get_logger().info("Sequence completed.")
            return False
        else:
            self.current_task = self.current_sequence[0]
            return True

    def run_task(self):
        """
        function that sends the current task command when it's triggered
        """
        if self.current_task is None:
            self.get_logger().warn("There is no further task available to be executed.")
            return False
        self.wait_for_trigger()
        self.send_task_command()
        return True

    def wait_for_trigger(self):
        """
        function that waits for the current trigger signal
        """
        while not self.stop:
            if self.current_task.trigger_signal is None:
                return True
            elif self.current_task.trigger_signal == self.hand_signal:
                self.get_logger().info("Trigger signal received.")
                return True
            else:
                time.sleep(0.1)
        return False


class Task:
    """ Class that defines the task properties. """

    def __init__(self, task_name: str, item_name: str, trigger_signal_name: str = None,
                 repeat_signal_name: str = None):
        self.name = task_name
        self.item = item_name
        self.trigger_signal = trigger_signal_name
        self.repeat_signal = repeat_signal_name

        self.valid_task_names = ["grab_from_hand", "place_in_hand", "grab_from_storage", "place_in_storage"]
        self.valid_item_names = ["cup", "cube"]

    def __str__(self):
        return f"Task({self.name}, {self.item}, {self.trigger_signal}, {self.repeat_signal})"


def main(args=None):
    """ Spin the task_controller node."""

    rclpy.init(args=args)
    try:
        task_controller = TaskController()
        # MultiThreadedExecutor executes callbacks with a thread pool. If num_threads is not
        # specified then num_threads will be multiprocessing.cpu_count() if it is implemented.
        # Otherwise, it will use a single thread. This executor will allow callbacks to happen in
        # parallel, however the MutuallyExclusiveCallbackGroup in DoubleTalker will only allow its
        # callbacks to be executed one at a time. The callbacks in Listener are free to execute in
        # parallel to the ones in DoubleTalker, however.
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(task_controller)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            task_controller.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
