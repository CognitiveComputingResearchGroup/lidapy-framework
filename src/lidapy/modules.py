from itertools import islice
from operator import attrgetter

import lidapy
import lidapy.strategy
from lidapy import LIDAModule
from lidapy import LocalMessageQueue
from lidapy import Task
from lidapy import Topic
from lidapy import logdebug, get_param
from lidapy.util import MsgUtils

# Common Topics
DORSAL_STREAM_TOPIC = Topic("dorsal_stream")
VENTRAL_STREAM_TOPIC = Topic("ventral_stream")
GLOBAL_BROADCAST_TOPIC = Topic("global_broadcast")
PERCEPTS_TOPIC = Topic("percepts")
EPISODES_TOPIC = Topic("episodes")
SPATIAL_MAPS_TOPIC = Topic("spatial_maps")
CANDIDATE_BEHAVIORS_TOPIC = Topic("candidate_behaviors")
SELECTED_BEHAVIORS_TOPIC = Topic("selected_behaviors")
DETECTED_FEATURES_TOPIC = Topic("detected_features")
WORKSPACE_CUES_TOPIC = Topic("workspace_cues")
WORKSPACE_COALITIONS_TOPIC = Topic("workspace_coalitions")


class Environment(LIDAModule):
    def __init__(self, tasks=None):
        super(Environment, self).__init__("environment", tasks)

    # TODO get topic names from conf file and generate topics \
    # TODO for the user based on msgtype specified as \
    # TODO      <(sensory/motor)-param-name> = <topic-name>, <msg-type>
    def initialize(self):
        pass


class ActionSelection(LIDAModule):
    def __init__(self, tasks=None):
        super(ActionSelection, self).__init__("action_selection", tasks)

        self.behavior_net = BehaviorNetwork()

        self.builtin_tasks = [
            Task("cand_recv_task", callback=self.receive_candidates),
            Task("behav_sel_task", callback=self.select_behavior),
            Task("behav_pub_task", callback=self.publish_selected_behavior),
            Task("decay_task", callback=self.decay),
            Task("learn_task", callback=self.learn)
        ]

        # The candidate_behaviors queue is used for the temporary storage of a set of instantiated
        # candidate behaviors.  These will be processed by a background task to determine
        # which (if any) will be selected for execution.
        self.candidate_behaviors = LocalMessageQueue()

        # The selected_behaviors queue is used for the temporary storage of a set of selected
        # behaviors.  These will be published to sensory motor memory for execution.
        self.selected_behaviors = LocalMessageQueue()

    def initialize(self):
        super(ActionSelection, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    # The callback for the receiver task
    def receive_candidates(self):
        next_candidates = CANDIDATE_BEHAVIORS_TOPIC.receive()
        if next_candidates is not None:
            self.candidate_behaviors.push(next_candidates)

    # The callback for the selector task
    def select_behavior(self):
        try:
            candidates = self.candidate_behaviors.pop()
        except IndexError:
            logdebug("No candidate behaviors received")
            candidates = None

        # Attempt to select a candidate behavior using a behavior network
        if candidates is not None:
            behavior = self.behavior_net.select_behavior(candidates, self.rate_in_hz)

            if behavior is not None:
                logdebug("Behavior selected: {}".format(behavior))
                self.selected_behaviors.push(behavior)

    # The callback for the publisher task
    def publish_selected_behavior(self):

        # Check for available selected behaviors
        try:
            behavior = self.selected_behaviors.pop()
        except IndexError:
            behavior = None

        # Publish selected behavior to selected behaviors queue for action
        # execution (if applicable)
        if behavior is not None:
            logdebug("Publishing selected behaviors for execution")
            SELECTED_BEHAVIORS_TOPIC.send(behavior)
        else:
            logdebug("No behaviors selected for execution")

    def decay(self):
        self.behavior_net.decay(self.rate_in_hz)

    def learn(self):
        global_broadcast = GLOBAL_BROADCAST_TOPIC.receive
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass


class BehaviorNetwork(object):
    def __init__(self):
        super(BehaviorNetwork, self).__init__()

        # A dictionary with the following structure:
        #    { Behavior Id 1: Behavior 1,
        #      Behavior Id 2: Behavior 2,
        #      ...}
        self.behaviors = dict()

        # A dictionary with the following structure:
        #    { Condition 1: {Behavior A, Behavior B, ...},
        #      Condition 2: {Behavior X, Behavior Y, ...},
        #      ...}
        self.behaviors_by_context = dict()

        # A dictionary with the following structure:
        #    { Condition 1: {Behavior A, Behavior B, ...},
        #      Condition 2: {Behavior X, Behavior Y, ...},
        #      ...}
        self.behaviors_by_adding_item = dict()

        # A dictionary with the following structure:
        #    { Condition 1: {Behavior A, Behavior B, ...},
        #      Condition 2: {Behavior X, Behavior Y, ...},
        #      ...}
        self.behaviors_by_deleting_item = dict()

        self.candidate_threshold = self.initial_candidate_threshold

        self.candidate_threshold_decay_strategy = lidapy.strategy.SigmoidDecayStrategy()
        self.behavior_decay_strategy = lidapy.strategy.SigmoidDecayStrategy()

    @property
    def initial_candidate_threshold(self):
        return get_param("initial_candidate_threshold", section="action_selection", default=0.9)

    @property
    def broadcast_excitation_factor(self):
        return get_param("broadcast_excitation_factor", section="action_selection", default=0.05)

    @property
    def successor_excitation_factor(self):
        return get_param("successor_excitation_factor", section="action_selection", default=0.04)

    @property
    def predecessor_excitation_factor(self):
        return get_param("predecessor_excitation_factor", section="action_selection", default=0.1)

    @property
    def conflictor_excitation_factor(self):
        return get_param("conflictor_excitation_factor", section="action_selection", default=0.04)

    @property
    def context_satisfaction_threshold(self):
        return get_param("context_satisfaction_threshold", section="action_selection", default=0.0)

    def select_behavior(self, behaviors, rate_in_hz):
        best_candidate = max(behaviors, key=attrgetter('activation'))

        # [Behavior Selected]
        if best_candidate.activation >= self.candidate_threshold:
            logdebug("Behavior selected: {}".format(best_candidate))

            # Remove activation from selected behavior
            best_candidate.activation = 0.0

            # Reset candidate threshold
            self.candidate_threshold = self.initial_candidate_threshold

            logdebug("Resetting candidate threshold to {}".format(self.candidate_threshold))

            return best_candidate

        # [No Selected Behavior]
        else:
            self.candidate_threshold \
                = self.candidate_threshold_decay_strategy.get_next_value(self.candidate_threshold, rate_in_hz)

            logdebug("No behavior selected.  Reducing candidate_threshold to {}".format(self.candidate_threshold))

            return None

    def decay(self, rate_in_hz):
        for behavior in self.behaviors:
            behavior.activation \
                = self.behavior_decay_strategy.get_next_value(behavior.activation,
                                                              rate_in_hz)
            if behavior.activation < behavior.r_threshold:
                self.remove_behavior(behavior)

    def add_behavior(self, behavior):
        if behavior is not None:
            self.behaviors[behavior.unique_id] = behavior

    def remove_behavior(self, behavior):
        if behavior is not None:
            try:
                self.behaviors.pop(behavior.unique_id)
            except KeyError:
                raise KeyError("Attempted to remove non-existent behavior [unique_id = {}] from BehaviorNetwork".format(
                    behavior.unique_id))


# class ConsciousContentsQueue(LIDAModule):
#     def __init__(self):
#         super(ConsciousContentsQueue, self).__init__("conscious_contents_queue")
#
#         self.queue = LocalMessageQueue(max_queue_size=get_param(self.name, "max_queue_size", 10))
#         self.tasks = [Task(name="broadcast_retriever", callback=self.retrieve_broadcast)]
#
#         self.last_broadcasts_service = \
#             lidapy.Service("get_last_n_broadcasts", self.process_last_n_broadcasts_request)
#
#     def retrieve_broadcast(self):
#         broadcast = GLOBAL_BROADCAST_TOPIC.next_msg
#
#         if broadcast is not None:
#             self.queue.push(broadcast)
#
#     def process_last_n_broadcasts_request(self, raw_request):
#         request = MsgUtils.deserialize(raw_request)
#         queue_size = len(self.queue)
#         if request.n > queue_size:
#             request.n = queue_size
#
#         response = CcqGetLastNBroadcastsResponse(request.n)
#
#         # TODO: Need to revisit this after changing the datatype of self.queue
#         response.last_n_broadcasts = list(islice(self.queue, queue_size - request.last_n, queue_size))
#
#         return response
#
#
# class CcqGetLastNBroadcastsRequest(object):
#     def __init__(self, n):
#         self.n = n
#
#
# class CcqGetLastNBroadcastsResponse(object):
#     def __init__(self, last_n_broadcasts):
#         self.last_n_broadcasts = last_n_broadcasts

class Memory(LIDAModule):
    def __init__(self, name, tasks=None):
        super(Memory, self).__init__(name, tasks)

        self.builtin_tasks = [
            Task(name='cue_receiver', callback=self.receive_cue),
            Task(name='learner', callback=self.receive_broadcast)
        ]

    def initialize(self):
        super(Memory, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    # TODO: Implement this
    def receive_cue(self):
        logdebug("Receiving workspace cue")

    def receive_broadcast(self):
        global_broadcast = GLOBAL_BROADCAST_TOPIC.receive
        if global_broadcast is not None:
            self.learn(global_broadcast)

    def learn(self, global_broadcast):
        pass


class EpisodicMemory(Memory):
    def __init__(self, tasks=None):
        super(EpisodicMemory, self).__init__("episodic_memory", tasks)

    def initialize(self):
        super(EpisodicMemory, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    def learn(self, global_broadcast):
        # TODO: Need to implement learning here
        pass


class GlobalWorkspace(LIDAModule):
    def __init__(self, tasks=None):
        super(GlobalWorkspace, self).__init__("global_workspace", tasks)

        self.builtin_tasks = [
            Task(name="coalition_receiver", callback=self.receive_coalitions)
        ]

    def initialize(self):
        super(GlobalWorkspace, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    # TODO: Implement this
    def receive_coalitions(self):
        logdebug("Receiving workspace coalitions")


class PerceptualAssociativeMemory(Memory):
    def __init__(self, tasks=None):
        super(PerceptualAssociativeMemory, self).__init__("perceptual_associative_memory", tasks)

    def initialize(self):
        super(PerceptualAssociativeMemory, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    def learn(self, global_broadcast):
        # TODO: Need to implement learning here
        pass


class ProceduralMemory(Memory):
    def __init__(self, tasks=None):
        super(ProceduralMemory, self).__init__("procedural_memory", tasks)

    def initialize(self):
        super(ProceduralMemory, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    def learn(self, global_broadcast):
        # TODO: Need to implement learning here
        pass


class SensoryMemory(Memory):
    def __init__(self, tasks=None):
        super(SensoryMemory, self).__init__("sensory_memory", tasks)

    def initialize(self):
        super(SensoryMemory, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    def learn(self, global_broadcast):
        # TODO: Need to implement learning here
        pass



class SensoryScene(object):
    def __init__(self, sensor):
        self.sensor = sensor

        self._layers = []

    def __setattr__(self, n, layer):
        # type: (int, SensorySceneLayer) -> None
        self._layers[n] = layer

    def __getattr__(self, n):
        # type: (int) -> SensorySceneLayer
        return self.layers[n]

    def append(self, layer):
        # type: (SensorySceneLayer) -> None
        self.layers.append(layer)

    def __len__(self):
        return len(self.layers)

    def update(self):
        for layer in self._layers:
            layer.update(self)


class SensorySceneLayer(object):
    def __init__(self, name, update_func):
        self.name = name

    def update(self, sensory_scene):
        pass


class SensorySceneCodelet(Task):
    def __init__(self,
                 input_layers,  # type: list
                 output_layers  # type: list
                 ):
        self.input_layers = input_layers
        self.output_layers = output_layers




class SensoryMotorMemory(Memory):
    def __init__(self):
        super(SensoryMotorMemory, self).__init__("sensory_motor_memory")

    def initialize(self):
        super(SensoryMotorMemory, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    def learn(self, global_broadcast):
        # TODO: Need to implement learning here
        pass


class SpatialMemory(Memory):
    def __init__(self, tasks=None):
        super(SpatialMemory, self).__init__("spatial_memory", tasks)

    def initialize(self):
        super(SpatialMemory, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    def learn(self, global_broadcast):
        # TODO: Need to implement learning here
        pass


class TransientEpisodicMemory(Memory):
    def __init__(self):
        super(TransientEpisodicMemory, self).__init__("transient_episodic_memory")

    def initialize(self):
        super(TransientEpisodicMemory, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    def learn(self, global_broadcast):
        # TODO: Need to implement learning here
        pass


class Workspace(Memory):
    def __init__(self, tasks=None):
        super(Workspace, self).__init__("workspace", tasks)

    def initialize(self):
        super(Workspace, self).initialize()

        # Added late to allow modification to built-ins before startup
        self.tasks += self.builtin_tasks

    def learn(self, global_broadcast):
        # TODO: Need to implement learning here
        pass

