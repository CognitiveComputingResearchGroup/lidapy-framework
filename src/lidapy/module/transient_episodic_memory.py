from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic


# Topics used by this module
EPISODES = FrameworkTopic("episodes")
WORKSPACE_CUES = FrameworkTopic("workspace_cues")
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")


class TransientEpisodicMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(TransientEpisodicMemory, self).__init__(**kwargs)

        self.add_publishers([EPISODES])
        self.add_subscribers([GLOBAL_BROADCAST,
                              WORKSPACE_CUES])

    @classmethod
    def get_module_name(cls):
        return "transient_episodic_memory"

    def call(self):
        pass
