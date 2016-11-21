from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# Topics used by this module
WORKSPACE_COALITIONS = FrameworkTopic("workspace_coalitions")
WORKSPACE_CUES = FrameworkTopic("workspace_cues")
PERCEPTS = FrameworkTopic("percepts")
SPATIAL_MAPS = FrameworkTopic("spatial_maps")
EPISODES = FrameworkTopic("episodes")
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")


class Workspace(FrameworkModule):
    def __init__(self):
        super(Workspace, self).__init__()

        self.add_publishers([WORKSPACE_COALITIONS,
                             WORKSPACE_CUES])
        self.add_subscribers([EPISODES,
                              GLOBAL_BROADCAST,
                              PERCEPTS,
                              SPATIAL_MAPS])

    @classmethod
    def get_module_name(cls):
        return "workspace"

    def call(self):
        percepts = PERCEPTS.subscriber.get_next_msg()

        if percepts is not None:
            pass
