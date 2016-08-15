from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "sensory_memory"

# Topics used by this module
DORSAL_STREAM_TOPIC = built_in_topics["dorsal_stream"]
VENTRAL_STREAM_TOPIC = built_in_topics["ventral_stream"]
DETECTED_FEATURES_TOPIC = built_in_topics["detected_features"]
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]


class SensoryMemory(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(SensoryMemory, self).__init__(name, **kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        self.add_publisher(DORSAL_STREAM_TOPIC)
        self.add_publisher(VENTRAL_STREAM_TOPIC)
        self.add_publisher(DETECTED_FEATURES_TOPIC)

    def add_subscribers(self):
        self.add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def call(self):
        pass
