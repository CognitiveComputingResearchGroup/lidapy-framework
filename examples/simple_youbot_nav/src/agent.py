#! /usr/bin/env python

import numpy
from random import uniform, choice
from sys import argv
from time import sleep

import lidapy
from lidapy import Config
from lidapy import LIDAProcess
from lidapy import Task
from lidapy import Topic
from sensor_msgs.msg import LaserScan
from simple_youbot_nav.msg import WheelCommand

# Topic definitions
ACTUATOR = Topic('/gazebo/commands', msg_type=WheelCommand)
DETECTED_FEATURES = Topic('detected_features')
SELECTED_ACTIONS = Topic('selected_actions', queue_size=2)
SENSOR = Topic('/gazebo/sensors/rayscan', msg_type=LaserScan, postprocessor=lambda m: m.ranges)

# Feature enumerations
BLOCKED = '1'
LEFT = '2'
RIGHT = '3'
FRONT = '4'
SPIN_LEFT = '5'
SPIN_RIGHT = '6'


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

    maxmin_dir, maxmin_range = max(mins.iteritems(), key=(lambda item: item[1]))
    maxmax_dir, maxmax_range = max(maxes.iteritems(), key=(lambda item: item[1]))
    maxavg_dir, maxavg_range = max(avgs.iteritems(), key=(lambda item: item[1]))

    if avgs[FRONT] < 0.25 and mins[FRONT] < 0.01:
        DETECTED_FEATURES.publish(BLOCKED)
    elif maxavg_dir == maxmax_dir == maxmin_dir:
        DETECTED_FEATURES.publish(maxavg_dir)
    else:
        candidate_dirs = [LEFT, RIGHT, FRONT]
        while candidate_dirs:
            candidate = choice(candidate_dirs)
            if maxes[candidate] > 1.0 and mins[candidate] > 0.25 and avgs[candidate] > 1.0:
                DETECTED_FEATURES.publish(candidate)
                break
            else:
                candidate_dirs.remove(candidate)

        # Unable to detect suitable direction
        if len(candidate_dirs) == 0:
            DETECTED_FEATURES.publish(choice([SPIN_LEFT, SPIN_RIGHT]))


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

        actions = {FRONT: Action(0.0, force, 0.1),
                   LEFT: Action(0.5, force, 0.1),
                   RIGHT: Action(-0.5, force, 0.1),
                   BLOCKED: Action(0.8, -force, 4),
                   SPIN_LEFT: Action(0.8, -force, 5),
                   SPIN_RIGHT: Action(-0.8, -force, 5)}

        SELECTED_ACTIONS.publish(actions[feature])


def execute_action():
    action = SELECTED_ACTIONS.next_msg
    if action:
        ACTUATOR.publish(action.command)
        sleep(action.duration)


# Initialize the lidapy framework
lidapy.init(Config(file_path=argv[1], use_param_service=True))

LIDAProcess('pam', tasks=[Task('path_detector', detect_path)]).start()
LIDAProcess('action_selection', tasks=[Task('action_selection', determine_action)]).start()
LIDAProcess('action_execution', tasks=[Task('action_execution', execute_action)]).start()
