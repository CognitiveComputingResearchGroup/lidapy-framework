from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "spatial_memory"

# Topics used by this module
SPATIAL_MAPS_TOPIC = FrameworkTopic("spatial_maps")
WORKSPACE_CUES_TOPIC = FrameworkTopic("workspace_cues")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")


class SpatialMemory(FrameworkModule):
    def __init__(self):
        super(SpatialMemory, self).__init__()

        self.add_publishers([SPATIAL_MAPS_TOPIC])
        self.add_subscribers([GLOBAL_BROADCAST_TOPIC,
                              WORKSPACE_CUES_TOPIC])

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def call(self):
        pass
