#!/usr/bin/env python

from numpy import average
from time import sleep

from collections import deque

import lidapy

from lidapy import Config
from lidapy import Task
from lidapy import Topic
from lidapy import Sensor

from lidapy.modules import PerceptualAssociativeMemory
from lidapy.modules import SensoryMemory
from lidapy.modules import SensoryMotorMemory
from lidapy.modules import DORSAL_STREAM_TOPIC
from lidapy.modules import VENTRAL_STREAM_TOPIC

from smach import State
from smach import StateMachine

from sensor_msgs.msg import LaserScan
from simple_youbot_nav.msg import WheelCommand

# Topic definitions
WHEEL_CMD_TOPIC = Topic("/gazebo/commands", msg_type=WheelCommand)


class BasicSensoryMemory(SensoryMemory):
    def __init__(self):
        super(BasicSensoryMemory, self).__init__()

    def initialize(self):
        # Add topics
        rayscan_topic = Topic("/gazebo/sensor/rayscan", msg_type=LaserScan)
        rayscan_sensor = Sensor("rayscan", topic=rayscan_topic)

        # Add sensory scene
        rayscan_scene = SensoryScene(sensor=rayscan_sensor)

        # Add background tasks
        self.add_task(Task("process_rayscan", callback=self.process_rayscan))

    def process_rayscan(self):
        data = self.sensors['rayscan'].data
        if data is not None:
            DORSAL_STREAM_TOPIC.publish(data)
            VENTRAL_STREAM_TOPIC.publish(data)


class BasicPerceptualAssociativeMemory(PerceptualAssociativeMemory):
    def __init__(self):
        super(BasicPerceptualAssociativeMemory, self).__init__()

        self.nodes()


class BasicSensoryMotorMemory(SensoryMotorMemory):
    def __init__(self):
        super(BasicSensoryMotorMemory, self).__init__()

        self.state_machine = self.create_state_machine()
        self.next_action = None

    def initialize(self):
        self.task_manager.add(Task("state_machine", callback=self.state_machine.execute))

    def receive_dorsal_stream(self):
        dorsal_stream = DORSAL_STREAM_TOPIC.subscriber.get_next_msg()

    def execute_action(self):
        if self.next_action is not None:
            WHEEL_CMD_TOPIC.publish(self.next_action)

    def create_state_machine(self):
        new_state_machine = StateMachine(outcomes=['success', 'failure'])
        with new_state_machine:
            StateMachine.add("SCAN", CheckEnv(self),
                             transitions={"clear_front": "FORWARD",
                                          "clear_left": "TURN_LEFT",
                                          "clear_right": "TURN_RIGHT",
                                          "blocked": "REVERSE",
                                          "unknown": "STOP"})
            StateMachine.add("FORWARD",
                             LinearMove(self, direction=LinearMove.FORWARD, duration=0.1),
                             transitions={"success": "success"})
            StateMachine.add("REVERSE",
                             Turn(self, angle=0.8, direction=LinearMove.REVERSE, duration=4),
                             transitions={"success": "success"})
            StateMachine.add("TURN_LEFT",
                             Turn(self, angle=0.8, direction=Turn.FORWARD, duration=0.1),
                             transitions={"success": "success"})
            StateMachine.add("TURN_RIGHT",
                             Turn(self, angle=-0.8, direction=Turn.FORWARD, duration=0.1),
                             transitions={"success": "success"})
            StateMachine.add("STOP",
                             LinearMove(self, direction=LinearMove.STOP, duration=0.1),
                             transitions={"success": "success"})

        return new_state_machine


# States for FSM
class CheckEnv(State):
    def __init__(self, smm):
        State.__init__(self,
                       outcomes=["clear_front",
                                 "clear_left",
                                 "clear_right",
                                 "blocked",
                                 "unknown"])
        self.smm = smm

    def execute(self, userdata):
        dorsal_stream_msg = self.smm.get_next_msg(DORSAL_STREAM_TOPIC)
        if dorsal_stream_msg is None:
            return "unknown"

        return self.get_path_state(dorsal_stream_msg)

    def get_path_state(self, msg):

        ranges = msg.value.ranges

        n_ranges = len(ranges)
        n_slice = int(n_ranges / 3.0)

        front_ranges = ranges[int((n_ranges - n_slice) / 2.0): int((n_ranges + n_slice) / 2.0)]
        left_ranges = ranges[(n_ranges - 1) - n_slice: (n_ranges - 1)]
        right_ranges = ranges[0:n_slice]

        # Blocked if any object within 0.1 meters of front range
        if min(front_ranges) < 0.1:
            return "blocked"

        averages = {"clear_front": average(front_ranges),
                    "clear_left": average(left_ranges),
                    "clear_right": average(right_ranges)}

        # Obstacle avoidance
        max_dir = "clear_front"
        while len(averages) > 0:
            max_dir = max(averages, key=averages.get)

            if max_dir == "clear_front" and min(front_ranges) < 0.5:
                averages.pop(max_dir)
            elif max_dir == "clear_left" and min(left_ranges) < 0.5:
                averages.pop(max_dir)
            elif max_dir == "clear_right" and min(right_ranges) < 0.5:
                averages.pop(max_dir)
            else:
                break

        # Blocked if all visible directions show less than 1 meter of available room
        if len(averages) == 0:
            return "blocked"

        # If not blocked, then move in the direction with the greatest average
        # range
        return max_dir


class LinearMove(State):
    FORWARD = 1
    STOP = 0
    REVERSE = -1

    def __init__(self, smm, direction, duration):
        State.__init__(self, outcomes=['success', 'failure'])

        self.smm = smm
        self.direction = direction
        self.duration = duration

    def get_next_cmd(self):
        cmd = WheelCommand()
        cmd.angle = 0.0
        cmd.force = self.direction * float(self.smm.config.get_param("sensory_motor_memory", "default_wheel_force"))
        return cmd

    def execute(self, userdata):
        self.smm.publish(WHEEL_CMD_TOPIC, self.get_next_cmd())
        sleep(self.duration)

        return 'success'


class Turn(State):
    FORWARD = 1
    REVERSE = -1

    def __init__(self, smm, angle, direction, duration):
        State.__init__(self, outcomes=['success', 'failure'])

        self.smm = smm
        self.angle = angle
        self.direction = direction
        self.duration = duration

    def get_next_cmd(self):
        cmd = WheelCommand()
        cmd.angle = self.angle
        cmd.force = self.direction * float(self.smm.config.get_param("sensory_motor_memory", "default_wheel_force"))
        return cmd

    def execute(self, userdata):
        self.smm.publish(WHEEL_CMD_TOPIC, self.get_next_cmd())
        sleep(self.duration)

        return 'success'


if __name__ == "__main__":
    lidapy.init(Config("../configs/agent.conf"))

    BasicSensoryMemory().start()
    BasicSensoryMotorMemory().start()
