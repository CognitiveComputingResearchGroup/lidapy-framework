from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic


# Topics used by this module
SELECTED_BEHAVIORS = FrameworkTopic("selected_behaviors")
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")
DORSAL_STREAM = FrameworkTopic("dorsal_stream")


class SensoryMotorMemory(FrameworkModule):
    def __init__(self):
        super(SensoryMotorMemory, self).__init__()

        self.add_subscribers([DORSAL_STREAM,
                              GLOBAL_BROADCAST,
                              SELECTED_BEHAVIORS])

    @classmethod
    def get_module_name(cls):
        return "sensory_motor_memory"

    def call(self):
        pass
