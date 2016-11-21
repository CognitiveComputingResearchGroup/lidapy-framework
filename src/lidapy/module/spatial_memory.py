from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic


# Topics used by this module
SPATIAL_MAPS = FrameworkTopic("spatial_maps")
WORKSPACE_CUES = FrameworkTopic("workspace_cues")
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")


class SpatialMemory(FrameworkModule):
    def __init__(self):
        super(SpatialMemory, self).__init__()

        self.add_publishers([SPATIAL_MAPS])
        self.add_subscribers([GLOBAL_BROADCAST,
                              WORKSPACE_CUES])

    @classmethod
    def get_module_name(cls):
        return "spatial_memory"

    def call(self):
        pass
