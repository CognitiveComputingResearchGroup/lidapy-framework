import lidapy
import numpy

from sys import argv
from time import sleep
from random import uniform, choice
from lidapy import Config
from lidapy import LIDAProcess
from lidapy import Task
from lidapy import Topic
from sensor_msgs.msg import LaserScan
from simple_youbot_nav.msg import WheelCommand

# Topic definitions
ACTUATOR = Topic('/gazebo/commands', msg_type=WheelCommand)
DETECTED_FEATURES = Topic('detected_features')
SELECTED_ACTIONS = Topic('selected_actions')
SENSOR = Topic('/gazebo/sensors/rayscan', msg_type=LaserScan, postprocessor=lambda m: m.ranges)

# Feature enumerations
BLOCKED = '1'
LEFT = '2'
RIGHT = '3'
FRONT = '4'


def detect_path():
    ranges = SENSOR.next_msg or []
    if not ranges:
        return

    n_ranges = len(ranges)
    n_slice = int(n_ranges / 3.0)

    front_ranges = ranges[int((n_ranges - n_slice) / 2.0): int((n_ranges + n_slice) / 2.0)]
    left_ranges = ranges[(n_ranges - 1) - n_slice: (n_ranges - 1)]
    right_ranges = ranges[0:n_slice]

    mins = {FRONT: min(front_ranges), LEFT: min(left_ranges), RIGHT: min(right_ranges)}
    maxes = {FRONT: max(front_ranges), LEFT: max(left_ranges), RIGHT: max(right_ranges)}
    avgs = {FRONT: numpy.average(front_ranges), LEFT: numpy.average(left_ranges), RIGHT: numpy.average(right_ranges)}

    minmin_dir, minmin_range = min(mins.iteritems(), key=(lambda item: item[1]))
    maxmax_dir, maxmax_range = max(maxes.iteritems(), key=(lambda item: item[1]))
    maxavg_dir, maxavg_range = max(avgs.iteritems(), key=(lambda item: item[1]))

    if mins[FRONT] < 0.25:
        DETECTED_FEATURES.publish(BLOCKED)
    elif avgs[FRONT] > 5.0:
        DETECTED_FEATURES.publish(FRONT)
    elif minmin_range < 0.5 and LEFT is minmin_dir:
        DETECTED_FEATURES.publish(RIGHT)
    elif minmin_range < 0.5 and RIGHT is minmin_dir:
        DETECTED_FEATURES.publish(LEFT)
    elif maxmax_dir == maxavg_dir:
        DETECTED_FEATURES.publish(maxmax_dir)
    else:
        DETECTED_FEATURES.publish(choice([FRONT, LEFT, RIGHT]))


class Action(object):
    def __init__(self, angle, force, duration):
        self.angle = angle
        self.force = force
        self.duration = duration

    @property
    def command(self):
        cmd = WheelCommand()
        cmd.angle = self.angle
        cmd.force = self.force
        return cmd


def determine_action():
    feature = DETECTED_FEATURES.next_msg
    if feature is not None:
        force = float(lidapy.get_param('wheel_force', section='action_execution'))

        actions = {FRONT: Action(0.0, force, uniform(0.3, 0.8)),
                   LEFT: Action(0.4, force, uniform(0.1, 0.3)),
                   RIGHT: Action(-0.4, force, uniform(0.1, 0.3)),
                   BLOCKED: Action(0.8, -force, uniform(0.5, 5))}

        SELECTED_ACTIONS.publish(actions[feature])


def execute_action():
    action = SELECTED_ACTIONS.next_msg
    if action:
        ACTUATOR.publish(action.command)
        sleep(action.duration)


# Initialize the lidapy framework
lidapy.init(Config(argv[1]))

LIDAProcess('pam', [Task('path_detector', detect_path)]).start()
LIDAProcess('action_selection', [Task('action_selection', determine_action)]).start()
LIDAProcess('action_execution', [Task('action_execution', execute_action)]).start()
