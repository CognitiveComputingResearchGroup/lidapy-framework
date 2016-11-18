from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "perceptual_associative_memory"

# Topics used by this module
PERCEPTS_TOPIC = FrameworkTopic("percepts")
VENTRAL_STREAM_TOPIC = FrameworkTopic("ventral_stream")
DETECTED_FEATURES_TOPIC = FrameworkTopic("detected_features")
WORKSPACE_CUES_TOPIC = FrameworkTopic("workspace_cues")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")


class PerceptualAssociativeMemory(FrameworkModule):
    def __init__(self):
        super(PerceptualAssociativeMemory, self).__init__()

        self.add_publishers([PERCEPTS_TOPIC])
        self.add_subscribers([DETECTED_FEATURES_TOPIC,
                              GLOBAL_BROADCAST_TOPIC,
                              VENTRAL_STREAM_TOPIC,
                              WORKSPACE_CUES_TOPIC])

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def call(self):
        detected_features = DETECTED_FEATURES_TOPIC.subscriber.get_next_msg()

        if detected_features is not None:
            active_percepts = detected_features

            PERCEPTS_TOPIC.publisher.publish(active_percepts)
