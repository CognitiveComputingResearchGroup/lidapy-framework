from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "sensory_motor_memory"

# Topics used by this module
SELECTED_BEHAVIORS_TOPIC = FrameworkTopic("selected_behaviors")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")
DORSAL_STREAM_TOPIC = FrameworkTopic("dorsal_stream")


class SensoryMotorMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(SensoryMotorMemory, self).__init__(**kwargs)

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
