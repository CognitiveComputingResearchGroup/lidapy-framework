from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "transient_episodic_memory"

# Topics used by this module
EPISODES_TOPIC = built_in_topics["episodes"]
WORKSPACE_CUES_TOPIC = built_in_topics["workspace_cues"]
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]


class TransientEpisodicMemory(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(TransientEpisodicMemory, self).__init__(name, **kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        self.add_publisher(EPISODES_TOPIC)

    def add_subscribers(self):
        self.add_subscriber(WORKSPACE_CUES_TOPIC)
        self.add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def call(self):
        pass
