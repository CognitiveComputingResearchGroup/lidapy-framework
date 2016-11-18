from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "action_selection"

# Topics used by this module
SELECTED_BEHAVIORS_TOPIC = FrameworkTopic("selected_behaviors")
CANDIDATE_BEHAVIORS_TOPIC = FrameworkTopic("candidate_behaviors")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")


class ActionSelection(FrameworkModule):
    def __init__(self):
        super(ActionSelection, self).__init__()

        self.add_publishers([SELECTED_BEHAVIORS_TOPIC])
        self.add_subscribers([CANDIDATE_BEHAVIORS_TOPIC,
                              GLOBAL_BROADCAST_TOPIC])

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def call(self):
        candidate_behaviors = CANDIDATE_BEHAVIORS_TOPIC.subscriber.get_next_msg()

        if candidate_behaviors is not None:
            selected_behaviors = candidate_behaviors

            SELECTED_BEHAVIORS_TOPIC.publisher.publish(selected_behaviors)
