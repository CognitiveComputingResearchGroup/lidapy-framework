#! /usr/bin/env python

import collections
import numpy
from copy import copy
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
SELECTED_ACTIONS = Topic('selected_actions')
SENSOR = Topic('/gazebo/sensors/rayscan', msg_type=LaserScan, postprocessor=lambda m: m.ranges)

# Feature enumerations
BLOCKED = '1'
LEFT = '2'
FRONT = '3'
RIGHT = '4'
OBSTACLE = '5'

# (name, direction, rayscan range) tuple
DirRange = collections.namedtuple(typename='DirRange', field_names=['name', 'dir', 'range'])


def detect_path():
    ranges = SENSOR.receive() or []
    if not ranges:
        return

    n_ranges = len(ranges)
    n_slice = int(n_ranges / 3.0)

    front_ranges = ranges[int((n_ranges - n_slice) / 2.0): int((n_ranges + n_slice) / 2.0)]
    left_ranges = ranges[(n_ranges - 1) - n_slice: (n_ranges - 1)]
    right_ranges = ranges[0:n_slice]

    absmins = {FRONT: min(front_ranges), LEFT: min(left_ranges), RIGHT: min(right_ranges)}
    absmaxs = {FRONT: max(front_ranges), LEFT: max(left_ranges), RIGHT: max(right_ranges)}
    absavgs = {FRONT: numpy.average(front_ranges), LEFT: numpy.average(left_ranges), RIGHT: numpy.average(right_ranges)}

    maxmin = DirRange('maxmin', *max(absmins.iteritems(), key=(lambda item: item[1])))
    maxmax = DirRange('maxmax', *max(absmaxs.iteritems(), key=(lambda item: item[1])))
    maxavg = DirRange('maxavg', *max(absavgs.iteritems(), key=(lambda item: item[1])))

    decision_criteria = [maxmin, maxmax, maxavg]

    # No good path options.  Rely on other feature detectors for evasive actions.
    if all(d.range < 0.7 for d in decision_criteria) or any(d.range < 0.4 for d in decision_criteria):
        detected_feature = OBSTACLE
    else:
        path_dirs = [d.dir for d in decision_criteria]
        # choose path direction that satisfies the most decision criteria
        detected_feature = collections.Counter(path_dirs).most_common(1)[0][0]

    DETECTED_FEATURES.send(detected_feature)


# history variables used by the "stuck" feature detector
no_progress_count = 0
prev_ranges = []


# Feature detector to determine if the agent is unable to move
# forward because of an obstacle.
def detect_stuck():
    global prev_ranges, no_progress_count

    ranges = SENSOR.receive() or []
    if not ranges:
        return

    # No history.
    elif not prev_ranges:
        pass

    # No sensor evidence of being stuck.  This is to prevent
    # confusion when traveling in a lane or when no surrounding
    # objects.
    elif all(r > .5 for r in ranges[20:-20]):
        pass

    else:

        if all(abs(r1 - r2) < 0.05 for r1, r2 in zip(prev_ranges, ranges)):
            no_progress_count += 1

        obstacle_detected = True if no_progress_count > 50 else False
        if obstacle_detected:
            lidapy.logwarn('Agent Stuck!')
            DETECTED_FEATURES.send(OBSTACLE)

    prev_ranges = copy(ranges)


class WheelAction(object):
    FORWARD = 1
    BACKWARD = -1

    def __init__(self, angle, direction, duration):
        self.angle = angle
        self.direction = direction
        self.duration = duration

    def __repr__(self):
        return '[angle: {}; direction: {}; duration: {}]'.format(self.angle, self.direction, self.duration)


MOVE_FORWARD = WheelAction(angle=0.0, direction=WheelAction.FORWARD, duration=0.1)
MOVE_BACKWARD = WheelAction(angle=0.8, direction=WheelAction.BACKWARD, duration=3)
MOVE_RIGHT = WheelAction(angle=-0.5, direction=WheelAction.FORWARD, duration=0.1)
MOVE_LEFT = WheelAction(angle=0.5, direction=WheelAction.FORWARD, duration=0.1)
STOP = WheelAction(angle=0.0, direction=WheelAction.FORWARD, duration=0.1)

feature_to_action_map = {FRONT: MOVE_FORWARD,
                         RIGHT: MOVE_RIGHT,
                         LEFT: MOVE_LEFT,
                         OBSTACLE: MOVE_BACKWARD}


def determine_action():
    feature = DETECTED_FEATURES.receive()
    if feature is not None:
        SELECTED_ACTIONS.send(feature_to_action_map[feature])


def execute_action():
    action = SELECTED_ACTIONS.receive(timeout=0.25)
    if not action:
        action = STOP

    wheel_cmd = WheelCommand()
    wheel_cmd.angle = action.angle
    wheel_cmd.force = action.direction * float(lidapy.get_param('wheel_force', section='action_execution'))

    ACTUATOR.send(wheel_cmd)

    # Sleep to let wheel command execution occur for the expected duration before
    # changing commands
    sleep(action.duration)


# Initialize the lidapy framework
lidapy.init(Config(file_path=argv[1], use_param_service=True), log_level=lidapy.LOG_LEVEL_DEBUG)

processes = [
    LIDAProcess('action_execution', [Task('action_execution', execute_action)]),
    LIDAProcess('action_selection', [Task('action_selector', determine_action)]),
    LIDAProcess('stuck_detector', [Task('stuck_detector', detect_stuck)]),
    LIDAProcess('path_detection', [Task('path_detector', detect_path)])
]
for p in processes:
    p.start()
    sleep(1)
