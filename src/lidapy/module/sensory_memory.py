from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "sensory_memory"

# Topics used by this module
DORSAL_STREAM_TOPIC = FrameworkTopic("dorsal_stream")
VENTRAL_STREAM_TOPIC = FrameworkTopic("ventral_stream")
DETECTED_FEATURES_TOPIC = FrameworkTopic("detected_features")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")


class SensoryMemory(FrameworkModule):
    def __init__(self):
        super(SensoryMemory, self).__init__()

        self.add_publishers([DETECTED_FEATURES_TOPIC,
                             DORSAL_STREAM_TOPIC,
                             VENTRAL_STREAM_TOPIC])
        self.add_subscribers([GLOBAL_BROADCAST_TOPIC])

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def call(self):
        pass
