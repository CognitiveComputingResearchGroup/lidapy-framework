from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "episodic_memory"

# Topics used by this module
EPISODES_TOPIC = FrameworkTopic("episodes")
WORKSPACE_CUES_TOPIC = FrameworkTopic("workspace_cues")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")


class EpisodicMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(EpisodicMemory, self).__init__(**kwargs)

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
