from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "procedural_memory"

# Topics used by this module
CANDIDATE_BEHAVIORS_TOPIC = FrameworkTopic("candidate_behaviors")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")


class ProceduralMemory(FrameworkModule):
    def __init__(self):
        super(ProceduralMemory, self).__init__()

        self.add_publishers([CANDIDATE_BEHAVIORS_TOPIC])
        self.add_subscribers([GLOBAL_BROADCAST_TOPIC])

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def call(self):
        global_broadcast = GLOBAL_BROADCAST_TOPIC.subscriber.get_next_msg()

        if global_broadcast is not None:
            candidate_behaviors = global_broadcast

            CANDIDATE_BEHAVIORS_TOPIC.publisher.publish(candidate_behaviors)
