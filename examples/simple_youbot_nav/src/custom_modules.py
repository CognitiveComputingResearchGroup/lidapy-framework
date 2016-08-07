from time import sleep

from ccrg_custom_msgs.msg import RayScanSensor, WheelCommand
from numpy import average
from smach import State, StateMachine

from lidapy.framework.msg import built_in_topics, FrameworkTopic
from lidapy.framework.shared import CognitiveContent
from lidapy.module.action_selection import ActionSelection
from lidapy.module.global_workspace import GlobalWorkspace
from lidapy.module.perceptual_associative_memory import PerceptualAssociativeMemory
from lidapy.module.procedural_memory import ProceduralMemory
from lidapy.module.sensory_memory import SensoryMemory
from lidapy.module.sensory_motor_memory import SensoryMotorMemory
from lidapy.module.workspace import Workspace
from lidapy.util import logger

# Topic definitions
RAYSCAN_TOPIC = FrameworkTopic("/gazebo/sensors/rayscan", RayScanSensor)
WHEELCMD_TOPIC = FrameworkTopic("/gazebo/commands", WheelCommand)
DORSAL_STREAM_TOPIC = built_in_topics["dorsal_stream"]
VENTRAL_STREAM_TOPIC = built_in_topics["ventral_stream"]
PERCEPTS_TOPIC = built_in_topics["percepts"]
WORKSPACE_COALITIONS_TOPIC = built_in_topics["workspace_coalitions"]
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]
CANDIDATE_BEHAVIORS_TOPIC = built_in_topics["candidate_behaviors"]
SELECTED_BEHAVIORS_TOPIC = built_in_topics["selected_behaviors"]


class BasicSensoryMemory(SensoryMemory):
    def __init__(self, **kwargs):
        super(BasicSensoryMemory, self).__init__(**kwargs)

    def add_subscribers(self):
        super(BasicSensoryMemory, self).add_subscribers()
        super(BasicSensoryMemory, self).add_subscriber(RAYSCAN_TOPIC)

    def get_next_msg(self, topic):
        return super(BasicSensoryMemory, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(BasicSensoryMemory, self).publish(topic, msg)

    def call(self):
        rayscan_data = self.get_next_msg(RAYSCAN_TOPIC)

        if rayscan_data is not None:
            self.publish(DORSAL_STREAM_TOPIC, CognitiveContent(rayscan_data))
            self.publish(VENTRAL_STREAM_TOPIC, CognitiveContent(rayscan_data))


class BasicPerceptualAssociativeMemory(PerceptualAssociativeMemory):
    def __init__(self, **kwargs):
        super(BasicPerceptualAssociativeMemory, self).__init__(**kwargs)

    def get_next_msg(self, topic):
        return super(BasicPerceptualAssociativeMemory, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(BasicPerceptualAssociativeMemory, self).publish(topic, msg)

    def call(self):
        ventral_stream_msg = self.get_next_msg(VENTRAL_STREAM_TOPIC)

        if ventral_stream_msg is not None:
            self.publish(PERCEPTS_TOPIC, ventral_stream_msg)


class BasicWorkspace(Workspace):
    def __init__(self, **kwargs):
        super(BasicWorkspace, self).__init__(**kwargs)

    def get_next_msg(self, topic):
        return super(BasicWorkspace, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(BasicWorkspace, self).publish(topic, msg)

    def call(self):
        percept_msg = self.get_next_msg(PERCEPTS_TOPIC)

        if percept_msg is not None:
            self.publish(WORKSPACE_COALITIONS_TOPIC, percept_msg)


class BasicGlobalWorkspace(GlobalWorkspace):
    def __init__(self, **kwargs):
        super(BasicGlobalWorkspace, self).__init__(**kwargs)

    def get_next_msg(self, topic):
        return super(BasicGlobalWorkspace, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(BasicGlobalWorkspace, self).publish(topic, msg)

    def call(self):
        coalitions_msg = self.get_next_msg(WORKSPACE_COALITIONS_TOPIC)

        if coalitions_msg is not None:
            self.publish(GLOBAL_BROADCAST_TOPIC, coalitions_msg)


class BasicProceduralMemory(ProceduralMemory):
    def __init__(self, **kwargs):
        super(BasicProceduralMemory, self).__init__(**kwargs)

    def get_next_msg(self, topic):
        return super(BasicProceduralMemory, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(BasicProceduralMemory, self).publish(topic, msg)

    def call(self):
        global_broadcast_msg = self.get_next_msg(GLOBAL_BROADCAST_TOPIC)

        if global_broadcast_msg is not None:
            self.publish(CANDIDATE_BEHAVIORS_TOPIC, global_broadcast_msg)


class BasicActionSelection(ActionSelection):
    def __init__(self, **kwargs):
        super(BasicActionSelection, self).__init__(**kwargs)

    def get_next_msg(self, topic):
        return super(BasicActionSelection, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(BasicActionSelection, self).publish(topic, msg)

    def call(self):
        candidate_behaviors_msg = self.get_next_msg(CANDIDATE_BEHAVIORS_TOPIC)

        if candidate_behaviors_msg is not None:
            self.publish(SELECTED_BEHAVIORS_TOPIC, candidate_behaviors_msg)


class BasicSensoryMotorMemory(SensoryMotorMemory):
    def __init__(self, **kwargs):
        super(BasicSensoryMotorMemory, self).__init__(**kwargs)

        self.stateMachine = self.create_state_machine()

    def add_publishers(self):
        super(BasicSensoryMotorMemory, self).add_publisher(WHEELCMD_TOPIC)

    def get_next_msg(self, topic):
        return super(SensoryMotorMemory, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(SensoryMotorMemory, self).publish(topic, msg)

    def call(self):
        self.stateMachine.execute()

    def create_state_machine(self):
        default_force = float(self.config.get_param("sensory_motor_memory", "default_wheel_force"))
        angle_left = 0.8
        angle_right = -0.8

        new_state_machine = StateMachine(outcomes=['complete', 'failure'])
        with new_state_machine:
            StateMachine.add("SCAN", CheckEnv(self), transitions={"clear_front": "FORWARD",
                                                                  "clear_left": "TURN_LEFT",
                                                                  "clear_right": "TURN_RIGHT",
                                                                  "blocked": "REVERSE",
                                                                  "unknown": "ALL_STOP"})
            StateMachine.add("FORWARD", Forward(self, 1, 0.1), transitions={"success": "complete"})
            StateMachine.add("REVERSE", Turn(self, angle_left, -1, 4), transitions={"success": "complete"})
            StateMachine.add("TURN_LEFT", Turn(self, angle_left, 1, 0.1), transitions={"success": "complete"})
            StateMachine.add("TURN_RIGHT", Turn(self, angle_right, 1, 0.1), transitions={"success": "complete"})
            StateMachine.add("ALL_STOP", Forward(self, 0.0, 0.1), transitions={"success": "complete"})

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
        logger.debug("Executing CheckEnv")

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

        max_dir = max(averages, key=averages.get)

        # Blocked if all visible directions show less than 1 meter of available room
        if averages[max_dir] < 1.0:
            return "blocked"

        # If not blocked, then move in the direction with the greatest average
        # range
        return max_dir


class Forward(State):
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
        logger.debug("Executing Forward")

        self.smm.publish(WHEELCMD_TOPIC, self.get_next_cmd())
        sleep(self.duration)

        return 'success'


class Turn(State):
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
        logger.debug("Executing Turn")

        self.smm.publish(WHEELCMD_TOPIC, self.get_next_cmd())
        sleep(self.duration)

        return 'success'
