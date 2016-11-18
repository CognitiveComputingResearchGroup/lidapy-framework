from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "workspace"

# Topics used by this module
WORKSPACE_COALITIONS_TOPIC = FrameworkTopic("workspace_coalitions")
WORKSPACE_CUES_TOPIC = FrameworkTopic("workspace_cues")
PERCEPTS_TOPIC = FrameworkTopic("percepts")
SPATIAL_MAPS_TOPIC = FrameworkTopic("spatial_maps")
EPISODES_TOPIC = FrameworkTopic("episodes")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")


class Workspace(FrameworkModule):
    def __init__(self):
        super(Workspace, self).__init__()

        self.add_publishers([WORKSPACE_COALITIONS_TOPIC,
                             WORKSPACE_CUES_TOPIC])
        self.add_subscribers([EPISODES_TOPIC,
                              GLOBAL_BROADCAST_TOPIC,
                              PERCEPTS_TOPIC,
                              SPATIAL_MAPS_TOPIC])

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def call(self):
        percepts = PERCEPTS_TOPIC.subscriber.get_next_msg()

        if percepts is not None:
            pass
