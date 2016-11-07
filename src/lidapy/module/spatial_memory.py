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
    def __init__(self, **kwargs):
        super(SpatialMemory, self).__init__(**kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        self.add_publisher(SPATIAL_MAPS_TOPIC)

    def add_subscribers(self):
        self.add_subscriber(WORKSPACE_CUES_TOPIC)
        self.add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def get_next_msg(self, topic):
        return self.get_next_msg(topic)

    def publish(self, topic, msg):
        self.publish(topic, msg)

    def call(self):
        pass
