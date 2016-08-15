from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "sensory_motor_memory"

# Topics used by this module
SELECTED_BEHAVIORS_TOPIC = built_in_topics["selected_behaviors"]
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]
DORSAL_STREAM_TOPIC = built_in_topics["dorsal_stream"]


class SensoryMotorMemory(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(SensoryMotorMemory, self).__init__(name, **kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        pass

    def add_subscribers(self):
        self.add_subscriber(SELECTED_BEHAVIORS_TOPIC)
        self.add_subscriber(GLOBAL_BROADCAST_TOPIC)
        self.add_subscriber(DORSAL_STREAM_TOPIC)

    def call(self):
        pass
