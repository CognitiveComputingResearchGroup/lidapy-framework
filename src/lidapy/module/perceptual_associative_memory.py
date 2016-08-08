from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "perceptual_associative_memory"

# Topics used by this module
PERCEPTS_TOPIC = built_in_topics["percepts"]
VENTRAL_STREAM_TOPIC = built_in_topics["ventral_stream"]
DETECTED_FEATURES_TOPIC = built_in_topics["detected_features"]
WORKSPACE_CUES_TOPIC = built_in_topics["workspace_cues"]
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]


class PerceptualAssociativeMemory(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(PerceptualAssociativeMemory, self).__init__(name, decayable=True,
                                                          cueable=True, **kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        super(PerceptualAssociativeMemory, self).add_publisher(PERCEPTS_TOPIC)

    def add_subscribers(self):
        super(PerceptualAssociativeMemory, self).add_subscriber(VENTRAL_STREAM_TOPIC)
        super(PerceptualAssociativeMemory, self).add_subscriber(DETECTED_FEATURES_TOPIC)
        super(PerceptualAssociativeMemory, self).add_subscriber(WORKSPACE_CUES_TOPIC)
        super(PerceptualAssociativeMemory, self).add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def get_next_msg(self, topic):
        return super(PerceptualAssociativeMemory, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(PerceptualAssociativeMemory, self).publish(topic, msg)

    def call(self):
        detected_features = self.get_next_msg(DETECTED_FEATURES_TOPIC)

        if detected_features is not None:
            active_percepts = detected_features

            self.publish(PERCEPTS_TOPIC, active_percepts)
