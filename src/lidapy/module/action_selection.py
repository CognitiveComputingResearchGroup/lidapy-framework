from operator import attrgetter

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.process import FrameworkBackgroundTask
from lidapy.framework.shared import FrameworkObject
from lidapy.framework.strategy import SigmoidDecayStrategy
from lidapy.util.comm import LocalMessageQueue


class ActionSelection(FrameworkModule):
    # Topic name constants
    SELECTED_BEHAVIORS_TOPIC_NAME = "selected_behaviors"
    CANDIDATE_BEHAVIORS_TOPIC_NAME = "candidate_behaviors"
    GLOBAL_BROADCAST_TOPIC_NAME = "global_broadcast"

    def __init__(self):
        super(ActionSelection, self).__init__()

        # Topics used by this module
        self.CANDIDATE_BEHAVIORS = FrameworkTopic(self.CANDIDATE_BEHAVIORS_TOPIC_NAME)
        self.SELECTED_BEHAVIORS = FrameworkTopic(self.SELECTED_BEHAVIORS_TOPIC_NAME)
        self.GLOBAL_BROADCAST = FrameworkTopic(self.GLOBAL_BROADCAST_TOPIC_NAME)

        self.default_max_queue_size \
            = self.config.get_type_or_global_param(self.get_module_name(), "max_queue_size", 10)

        # The candidate_behaviors queue is used for the temporary storage of a set of instantiated
        # candidate behaviors.  These will be processed by a background task to determine
        # which (if any) will be selected for execution.
        self.candidate_behaviors_max_queue_size \
            = self.config.get_param(self.get_module_name(), "candidate_behaviors_max_queue_size",
                                    self.default_max_queue_size)

        self.candidate_behaviors_queue \
            = LocalMessageQueue(max_queue_size=self.candidate_behaviors_max_queue_size)

        # The selected_behaviors queue is used for the temporary storage of a set of selected
        # behaviors.  These will be published to sensory motor memory for execution.
        self.selected_behaviors_max_queue_size \
            = self.config.get_param(self.get_module_name(), "selected_behaviors_max_queue_size",
                                    self.default_max_queue_size)

        self.selected_behaviors_queue \
            = LocalMessageQueue(max_queue_size=self.selected_behaviors_max_queue_size)

        self.behavior_net = BehaviorNetwork()

        self.receiver_task \
            = FrameworkBackgroundTask(name="receiver",
                                      callback=self.receive_candidates)

        self.selector_task \
            = FrameworkBackgroundTask(name="selector",
                                      callback=self.select_behavior_from_candidates)

        self.publisher_task \
            = FrameworkBackgroundTask(name="publisher",
                                      callback=self.publish_selected_behavior)

        self.decayer_task \
            = FrameworkBackgroundTask(name="decayer",
                                      callback=self.decay_behaviors)

        self.learner_task \
            = FrameworkBackgroundTask(name="learner",
                                      callback=self.learn)

    @classmethod
    def get_module_name(cls):
        return "action_selection"

    # This method is invoked (only once) at module execution start
    def initialize(self):
        super(ActionSelection, self).initialize()

        self.add_publishers([self.SELECTED_BEHAVIORS])
        self.add_subscribers([self.CANDIDATE_BEHAVIORS, self.GLOBAL_BROADCAST])
        self.add_background_tasks([self.receiver_task,
                                   self.selector_task,
                                   self.publisher_task,
                                   self.decayer_task,
                                   self.learner_task])

        self.launch_background_tasks()

    # This method is invoked when module execution completes
    def finalize(self):
        super(ActionSelection, self).finalize()

    # This method is invoked at regular intervals during module execution
    def update_status(self):
        super(ActionSelection, self).update_status()

    # This method is invoked by the agent starter to initiate module execution
    def start(self):
        super(ActionSelection, self).start()

    # The callback for the receiver task
    def receive_candidates(self):
        next_candidates = self.subscribers[self.CANDIDATE_BEHAVIORS_TOPIC_NAME].get_next_msg()
        if next_candidates is not None:
            self.candidate_behaviors_queue.push(next_candidates)

    # The callback for the selector task
    def select_behavior_from_candidates(self):
        # Check for available candidate behaviors
        try:
            candidate_behaviors = self.candidate_behaviors_queue.pop()
        except IndexError:
            self.logger.debug("No candidate behaviors received")
            candidate_behaviors = None

        # Attempt to select a candidate behavior using a behavior network
        if candidate_behaviors is not None:
            selected_behavior = self.behavior_net.select_behavior(candidate_behaviors, self.rate_in_hz)

            if selected_behavior is not None:
                self.logger.debug("Behavior selected: {}".format(selected_behavior))
                self.selected_behaviors_queue.push(selected_behavior)

    # The callback for the publisher task
    def publish_selected_behavior(self):

        # Check for available selected behaviors
        try:
            next_behavior = self.selected_behaviors_queue.pop()
        except IndexError:
            next_behavior = None

        # Publish selected behavior to selected behaviors queue for action
        # execution (if applicable)
        if next_behavior is not None:
            self.logger.debug("Publishing selected behaviors for execution")
            self.publishers[self.SELECTED_BEHAVIORS_TOPIC_NAME].publish(next_behavior)
        else:
            self.logger.debug("No behaviors selected for execution")

    def decay_behaviors(self):
        self.behavior_net.decay(self.rate_in_hz)

    def learn(self):
        global_broadcast = self.subscribers[self.GLOBAL_BROADCAST_TOPIC_NAME].get_next_msg()
        if global_broadcast is not None:
            # TODO: Need to implement learning here
            pass


class BehaviorNetwork(FrameworkObject):
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

        self.candidate_threshold_decay_strategy = SigmoidDecayStrategy()
        self.behavior_decay_strategy = SigmoidDecayStrategy()

    @property
    def initial_candidate_threshold(self):
        return self.config.get_param("action_selection", "initial_candidate_threshold", default_value=0.9)

    @property
    def broadcast_excitation_factor(self):
        return self.config.get_param("action_selection", "broadcast_excitation_factor", default_value=0.05)

    @property
    def successor_excitation_factor(self):
        return self.config.get_param("action_selection", "successor_excitation_factor", default_value=0.04)

    @property
    def predecessor_excitation_factor(self):
        return self.config.get_param("action_selection", "predecessor_excitation_factor", default_value=0.1)

    @property
    def conflictor_excitation_factor(self):
        return self.config.get_param("action_selection", "conflictor_excitation_factor", default_value=0.04)

    @property
    def context_satisfaction_threshold(self):
        return self.config.get_param("action_selection", "context_satisfaction_threshold", default_value=0.0)

    def select_behavior(self, behaviors, rate_in_hz):
        best_candidate = max(behaviors, key=attrgetter('activation'))

        # [Behavior Selected]
        if best_candidate.activation >= self.candidate_threshold:
            self.logger.debug("Behavior selected: {}".format(best_candidate))

            # Remove activation from selected behavior
            best_candidate.activation = 0.0

            # Reset candidate threshold
            self.candidate_threshold = self.initial_candidate_threshold

            self.logger.debug("Resetting candidate threshold to {}".format(self.candidate_threshold))

            return best_candidate

        # [No Selected Behavior]
        else:
            self.candidate_threshold \
                = self.candidate_threshold_decay_strategy.get_next_value(self.candidate_threshold, rate_in_hz)

            self.logger.debug(
                "No behavior selected.  Reducing candidate_threshold to {}".format(self.candidate_threshold))

            return None

    def decay(self, rate_in_hz):
        for behavior in self.behaviors:
            behavior.activation \
                = self.behavior_decay_strategy.get_next_value(behavior.activation,
                                                              rate_in_hz)
            if behavior.activation < behavior.removal_threshold:
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
