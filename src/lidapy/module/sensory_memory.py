from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic


# Topics used by this module
DORSAL_STREAM = FrameworkTopic("dorsal_stream")
VENTRAL_STREAM = FrameworkTopic("ventral_stream")
DETECTED_FEATURES = FrameworkTopic("detected_features")
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")


class SensoryMemory(FrameworkModule):
    def __init__(self):
        super(SensoryMemory, self).__init__()

        self.add_publishers([DETECTED_FEATURES,
                             DORSAL_STREAM,
                             VENTRAL_STREAM])
        self.add_subscribers([GLOBAL_BROADCAST])

    @classmethod
    def get_module_name(cls):
        return "sensory_memory"

    def call(self):
        pass
