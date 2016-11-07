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
    def __init__(self, **kwargs):
        super(PerceptualAssociativeMemory, self).__init__(**kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        self.add_publisher(PERCEPTS_TOPIC)

    def add_subscribers(self):
        self.add_subscriber(VENTRAL_STREAM_TOPIC)
        self.add_subscriber(DETECTED_FEATURES_TOPIC)
        self.add_subscriber(WORKSPACE_CUES_TOPIC)
        self.add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def call(self):
        detected_features = self.get_next_msg(DETECTED_FEATURES_TOPIC)

        if detected_features is not None:
            active_percepts = detected_features

            self.publish(PERCEPTS_TOPIC, active_percepts)
