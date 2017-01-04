from itertools import islice
from operator import attrgetter

import lidapy
import lidapy.strategy
from lidapy import LIDAModule
from lidapy import LocalMessageQueue
from lidapy import Task
from lidapy import logdebug, get_param
from lidapy.util import MsgUtils

# Common Topics
DORSAL_STREAM_TOPIC = lidapy.Topic("dorsal_stream")
VENTRAL_STREAM_TOPIC = lidapy.Topic("ventral_stream")
GLOBAL_BROADCAST_TOPIC = lidapy.Topic("global_broadcast")
PERCEPTS_TOPIC = lidapy.Topic("percepts")
EPISODES_TOPIC = lidapy.Topic("episodes")
SPATIAL_MAPS_TOPIC = lidapy.Topic("spatial_maps")
CANDIDATE_BEHAVIORS_TOPIC = lidapy.Topic("candidate_behaviors")
SELECTED_BEHAVIORS_TOPIC = lidapy.Topic("selected_behaviors")
DETECTED_FEATURES_TOPIC = lidapy.Topic("detected_features")
WORKSPACE_CUES_TOPIC = lidapy.Topic("workspace_cues")
WORKSPACE_COALITIONS_TOPIC = lidapy.Topic("workspace_coalitions")


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
        next_candidates = CANDIDATE_BEHAVIORS_TOPIC.next_msg
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
            SELECTED_BEHAVIORS_TOPIC.publish(behavior)
        else:
            logdebug("No behaviors selected for execution")

    def decay(self):
        self.behavior_net.decay(self.rate_in_hz)

    def learn(self):
        global_broadcast = GLOBAL_BROADCAST_TOPIC.next_msg
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


class ConsciousContentsQueue(LIDAModule):
    def __init__(self):
        super(ConsciousContentsQueue, self).__init__("conscious_contents_queue")

        self.queue = LocalMessageQueue(max_queue_size=get_param(self.name, "max_queue_size", 10))
        self.tasks = [Task(name="broadcast_retriever", callback=self.retrieve_broadcast)]

        self.last_broadcasts_service = \
            lidapy.Service("get_last_n_broadcasts", self.process_last_n_broadcasts_request)

    def retrieve_broadcast(self):
        broadcast = GLOBAL_BROADCAST_TOPIC.next_msg

        if broadcast is not None:
            self.queue.push(broadcast)

    def process_last_n_broadcasts_request(self, raw_request):
        request = MsgUtils.deserialize(raw_request)
        queue_size = len(self.queue)
        if request.n > queue_size:
            request.n = queue_size

        response = CcqGetLastNBroadcastsResponse(request.n)

        # TODO: Need to revisit this after changing the datatype of self.queue
        response.last_n_broadcasts = list(islice(self.queue, queue_size - request.last_n, queue_size))

        return response


class CcqGetLastNBroadcastsRequest(object):
    def __init__(self, n):
        self.n = n


class CcqGetLastNBroadcastsResponse(object):
    def __init__(self, last_n_broadcasts):
        self.last_n_broadcasts = last_n_broadcasts


class EpisodicMemory(LIDAModule):
    def __init__(self):
        super(EpisodicMemory, self).__init__("episodic_memory")

        self.tasks = [Task(name="cue_receiver", callback=self.receive_cue),
                      Task(name="learner", callback=self.learn)]

    # TODO: Implement this
    def receive_cue(self):
        logdebug("Receiving workspace cue")

    def learn(self):
        global_broadcast = GLOBAL_BROADCAST_TOPIC.next_msg
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass


class GlobalWorkspace(LIDAModule):
    def __init__(self):
        super(GlobalWorkspace, self).__init__("global_workspace")

        self.GLOBAL_BROADCAST = lidapy.Topic("global_broadcast")
        self.WORKSPACE_COALITIONS = lidapy.Topic("workspace_coalitions")

        self.coalition_receiver_task \
            = Task(name="coalition_receiver",
                   callback=self.receive_coalitions)

    # TODO: Implement this
    def receive_coalitions(self):
        logdebug("Receiving workspace coalitions")


class PerceptualAssociativeMemory(LIDAModule):
    def __init__(self):
        super(PerceptualAssociativeMemory, self).__init__("perceptual_associative_memory")

        self.learner_task \
            = Task(name="learner",
                   callback=self.learn)

    def initialize(self):
        self.add_publishers([self.PERCEPTS])
        self.add_subscribers([self.DETECTED_FEATURES,
                              self.GLOBAL_BROADCAST,
                              self.VENTRAL_STREAM,
                              self.WORKSPACE_CUES])
        self.add_tasks([self.learner_task])

        self.launch_tasks()

    def learn(self):
        global_broadcast = self.GLOBAL_BROADCAST.next_msg
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass


class ProceduralMemory(LIDAModule):
    def __init__(self):
        super(ProceduralMemory, self).__init__("procedural_memory")

        # Topics used by this modules
        self.CANDIDATE_BEHAVIORS = lidapy.Topic("candidate_behaviors")
        self.GLOBAL_BROADCAST = lidapy.Topic("global_broadcast")

        self.learner_task \
            = Task(name="learner",
                   callback=self.learn)

    # This method is invoked (only once) at modules execution start
    def initialize(self):
        super(ProceduralMemory, self).initialize()

        self.add_publishers([self.CANDIDATE_BEHAVIORS])
        self.add_subscribers([self.GLOBAL_BROADCAST])
        self.add_tasks([self.learner_task])

        self.launch_tasks()

    def learn(self):
        global_broadcast = self.GLOBAL_BROADCAST.next_msg
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass


class SensoryMemory(LIDAModule):
    def __init__(self):
        super(SensoryMemory, self).__init__("sensory_memory")

        self.sensor_scene = SensoryScene()

    # This method is invoked (only once) at modules execution start
    def initialize(self):
        super(SensoryMemory, self).initialize()

        self.add_publishers([self.DETECTED_FEATURES,
                             self.DORSAL_STREAM,
                             self.VENTRAL_STREAM])
        self.add_subscribers([self.GLOBAL_BROADCAST])
        self.add_tasks([self.learner_task])

        self.launch_tasks()

    def learn(self):
        global_broadcast = self.GLOBAL_BROADCAST.next_msg
        if global_broadcast is not None:
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


class SensoryMotorMemory(LIDAModule):
    def __init__(self):
        super(SensoryMotorMemory, self).__init__("sensory_motor_memory")

        # Topics used by this modules
        self.DORSAL_STREAM = lidapy.Topic("dorsal_stream")
        self.GLOBAL_BROADCAST = lidapy.Topic("global_broadcast")
        self.SELECTED_BEHAVIORS = lidapy.Topic("selected_behaviors")

        self.learner_task \
            = Task(name="learner",
                   callback=self.learn)

    # This method is invoked (only once) at modules execution start
    def initialize(self):
        super(SensoryMotorMemory, self).initialize()

        self.add_subscribers([self.DORSAL_STREAM,
                              self.GLOBAL_BROADCAST,
                              self.SELECTED_BEHAVIORS])
        self.add_tasks([self.learner_task])

        self.launch_tasks()

    def learn(self):
        global_broadcast = self.GLOBAL_BROADCAST.next_msg
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass


class SpatialMemory(LIDAModule):
    def __init__(self):
        super(SpatialMemory, self).__init__("spatial_memory")

        # Topics used by this modules
        self.SPATIAL_MAPS = lidapy.Topic("spatial_maps")
        self.GLOBAL_BROADCAST = lidapy.Topic("global_broadcast")
        self.WORKSPACE_CUES = lidapy.Topic("workspace_cues")

        self.learner_task \
            = Task(name="learner",
                   callback=self.learn)

    # This method is invoked (only once) at modules execution start
    def initialize(self):
        super(SpatialMemory, self).initialize()

        self.add_publishers([self.SPATIAL_MAPS])
        self.add_subscribers([self.GLOBAL_BROADCAST,
                              self.WORKSPACE_CUES])
        self.add_tasks([self.learner_task])

        self.launch_tasks()

    def learn(self):
        global_broadcast = self.GLOBAL_BROADCAST.next_msg
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass


class TransientEpisodicMemory(LIDAModule):
    def __init__(self):
        super(TransientEpisodicMemory, self).__init__("transient_episodic_memory")

        # Topics used by this modules
        self.EPISODES = lidapy.Topic("episodes")
        self.GLOBAL_BROADCAST = lidapy.Topic("global_broadcast")
        self.WORKSPACE_CUES = lidapy.Topic("workspace_cues")

        self.learner_task \
            = Task(name="learner",
                   callback=self.learn)

    # This method is invoked (only once) at modules execution start
    def initialize(self):
        super(TransientEpisodicMemory, self).initialize()

        self.add_publishers([self.EPISODES])
        self.add_subscribers([self.GLOBAL_BROADCAST,
                              self.WORKSPACE_CUES])
        self.add_tasks([self.learner_task])

        self.launch_tasks()

    def learn(self):
        global_broadcast = self.GLOBAL_BROADCAST.next_msg
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass


class Workspace(LIDAModule):
    def __init__(self):
        super(Workspace, self).__init__("workspace")

        # Topics used by this modules
        self.WORKSPACE_COALITIONS = lidapy.Topic("workspace_coalitions")
        self.WORKSPACE_CUES = lidapy.Topic("workspace_cues")
        self.EPISODES = lidapy.Topic("episodes")
        self.GLOBAL_BROADCAST = lidapy.Topic("global_broadcast")
        self.PERCEPTS = lidapy.Topic("percepts")
        self.SPATIAL_MAPS = lidapy.Topic("spatial_maps")

        self.learner_task \
            = Task(name="learner",
                   callback=self.learn)

    # This method is invoked (only once) at modules execution start
    def initialize(self):
        super(Workspace, self).initialize()

        self.add_publishers([self.WORKSPACE_COALITIONS,
                             self.WORKSPACE_CUES])
        self.add_subscribers([self.EPISODES,
                              self.GLOBAL_BROADCAST,
                              self.PERCEPTS,
                              self.SPATIAL_MAPS])
        self.add_tasks([self.learner_task])

        self.launch_tasks()

    def learn(self):
        global_broadcast = self.GLOBAL_BROADCAST.next_msg
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass
