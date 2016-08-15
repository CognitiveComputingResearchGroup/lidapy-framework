from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "episodic_memory"

# Topics used by this module
EPISODES_TOPIC = built_in_topics["episodes"]
WORKSPACE_CUES_TOPIC = built_in_topics["workspace_cues"]
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]


class EpisodicMemory(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(EpisodicMemory, self).__init__(name, **kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        super(EpisodicMemory, self).add_publisher(EPISODES_TOPIC)

    def add_subscribers(self):
        super(EpisodicMemory, self).add_subscriber(WORKSPACE_CUES_TOPIC)
        super(EpisodicMemory, self).add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def get_next_msg(self, topic):
        return super(EpisodicMemory, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(EpisodicMemory, self).publish(topic, msg)

    def call(self):
        pass
