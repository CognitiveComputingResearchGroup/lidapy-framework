from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic


# Topics used by this module
PERCEPTS = FrameworkTopic("percepts")
VENTRAL_STREAM = FrameworkTopic("ventral_stream")
DETECTED_FEATURES = FrameworkTopic("detected_features")
WORKSPACE_CUES = FrameworkTopic("workspace_cues")
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")


class PerceptualAssociativeMemory(FrameworkModule):
    def __init__(self):
        super(PerceptualAssociativeMemory, self).__init__()

        self.add_publishers([PERCEPTS])
        self.add_subscribers([DETECTED_FEATURES,
                              GLOBAL_BROADCAST,
                              VENTRAL_STREAM,
                              WORKSPACE_CUES])

    @classmethod
    def get_module_name(cls):
        return "perceptual_associative_memory"

    def call(self):
        detected_features = DETECTED_FEATURES.subscriber.get_next_msg()

        if detected_features is not None:
            active_percepts = detected_features

            PERCEPTS.publisher.publish(active_percepts)
