from numpy import average
from time import sleep

from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.shared import CognitiveContent
from lidapy.module.sensory_memory import SensoryMemory, DORSAL_STREAM_TOPIC, VENTRAL_STREAM_TOPIC
from lidapy.module.sensory_motor_memory import SensoryMotorMemory
from sensor_msgs.msg import LaserScan
from simple_youbot_nav.msg import WheelCommand
from smach import State, StateMachine


# Topic definitions
RAYSCAN_TOPIC = FrameworkTopic("/gazebo/sensors/rayscan", msg_type=LaserScan)
WHEELCMD_TOPIC = FrameworkTopic("/gazebo/commands", msg_type=WheelCommand)


class BasicSensoryMemory(SensoryMemory):
    def __init__(self, **kwargs):
        super(BasicSensoryMemory, self).__init__(**kwargs)

    def add_subscribers(self):
        super(BasicSensoryMemory, self).add_subscribers()

        self.add_subscriber(RAYSCAN_TOPIC)

    def call(self):
        rayscan_data = self.get_next_msg(RAYSCAN_TOPIC)

        if rayscan_data is not None:
            self.publish(DORSAL_STREAM_TOPIC, CognitiveContent(rayscan_data))
            self.publish(VENTRAL_STREAM_TOPIC, CognitiveContent(rayscan_data))


class BasicSensoryMotorMemory(SensoryMotorMemory):
    def __init__(self, **kwargs):
        super(BasicSensoryMotorMemory, self).__init__(**kwargs)

        self.stateMachine = self.create_state_machine()

    def add_publishers(self):
        super(BasicSensoryMotorMemory, self).add_publishers()

        self.add_publisher(WHEELCMD_TOPIC)

    def call(self):
        self.stateMachine.execute()

    def create_state_machine(self):
        new_state_machine = StateMachine(outcomes=['success', 'failure'])
        with new_state_machine:
            StateMachine.add("SCAN", CheckEnv(self, logger=self.logger),
                             transitions={"clear_front": "FORWARD",
                                          "clear_left": "TURN_LEFT",
                                          "clear_right": "TURN_RIGHT",
                                          "blocked": "REVERSE",
                                          "unknown": "STOP"})
            StateMachine.add("FORWARD",
                             LinearMove(self, direction=LinearMove.FORWARD, duration=0.1, logger=self.logger),
                             transitions={"success": "success"})
            StateMachine.add("REVERSE",
                             Turn(self, angle=0.8, direction=LinearMove.REVERSE, duration=4, logger=self.logger),
                             transitions={"success": "success"})
            StateMachine.add("TURN_LEFT",
                             Turn(self, angle=0.8, direction=Turn.FORWARD, duration=0.1, logger=self.logger),
                             transitions={"success": "success"})
            StateMachine.add("TURN_RIGHT",
                             Turn(self, angle=-0.8, direction=Turn.FORWARD, duration=0.1, logger=self.logger),
                             transitions={"success": "success"})
            StateMachine.add("STOP",
                             LinearMove(self, direction=LinearMove.STOP, duration=0.1, logger=self.logger),
                             transitions={"success": "success"})

        return new_state_machine


# States for FSM
class CheckEnv(State):
    def __init__(self, smm, logger):
        State.__init__(self,
                       outcomes=["clear_front",
                                 "clear_left",
                                 "clear_right",
                                 "blocked",
                                 "unknown"])
        self.smm = smm
        self.logger = logger

    def execute(self, userdata):
        self.logger.debug("Executing CheckEnv")

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

    def __init__(self, smm, direction, duration, logger):
        State.__init__(self, outcomes=['success', 'failure'])

        self.smm = smm
        self.direction = direction
        self.duration = duration
        self.logger = logger

    def get_next_cmd(self):
        cmd = WheelCommand()
        cmd.angle = 0.0
        cmd.force = self.direction * float(self.smm.config.get_param("sensory_motor_memory", "default_wheel_force"))
        return cmd

    def execute(self, userdata):
        self.logger.debug("Executing Forward")

        self.smm.publish(WHEELCMD_TOPIC, self.get_next_cmd())
        sleep(self.duration)

        return 'success'


class Turn(State):
    FORWARD = 1
    REVERSE = -1

    def __init__(self, smm, angle, direction, duration, logger):
        State.__init__(self, outcomes=['success', 'failure'])

        self.smm = smm
        self.angle = angle
        self.direction = direction
        self.duration = duration
        self.logger = logger

    def get_next_cmd(self):
        cmd = WheelCommand()
        cmd.angle = self.angle
        cmd.force = self.direction * float(self.smm.config.get_param("sensory_motor_memory", "default_wheel_force"))
        return cmd

    def execute(self, userdata):
        self.logger.debug("Executing Turn")

        self.smm.publish(WHEELCMD_TOPIC, self.get_next_cmd())
        sleep(self.duration)

        return 'success'
