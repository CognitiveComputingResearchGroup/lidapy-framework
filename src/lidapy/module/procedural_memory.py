from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "procedural_memory"

# Topics used by this module
CANDIDATE_BEHAVIORS_TOPIC = built_in_topics["candidate_behaviors"]
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]


class ProceduralMemory(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(ProceduralMemory, self).__init__(name, decayable=True, **kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        super(ProceduralMemory, self).add_publisher(CANDIDATE_BEHAVIORS_TOPIC)

    def add_subscribers(self):
        super(ProceduralMemory, self).add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def get_next_msg(self, topic):
        return super(ProceduralMemory, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(ProceduralMemory, self).publish(topic, msg)

    def call(self):
        global_broadcast = self.get_next_msg(GLOBAL_BROADCAST_TOPIC)

        if global_broadcast is not None:
            candidate_behaviors = global_broadcast

            self.publish(CANDIDATE_BEHAVIORS_TOPIC, candidate_behaviors)
