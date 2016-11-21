from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.shared import Activatable


# Topics used by this module
from lidapy.framework.shared import FrameworkObject

CANDIDATE_BEHAVIORS = FrameworkTopic("candidate_behaviors")
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")


class ProceduralMemory(FrameworkModule):
    def __init__(self):
        super(ProceduralMemory, self).__init__()

        self.add_publishers([CANDIDATE_BEHAVIORS])
        self.add_subscribers([GLOBAL_BROADCAST])

    @classmethod
    def get_module_name(cls):
        return "procedural_memory"

    def call(self):
        global_broadcast = GLOBAL_BROADCAST.subscriber.get_next_msg()

        if global_broadcast is not None:
            candidate_behaviors = global_broadcast

            CANDIDATE_BEHAVIORS.publisher.publish(candidate_behaviors)


class Scheme(Activatable):
    def __init__(self):
        super(Scheme, self).__init__()


class Behavior(Activatable):
    def __init__(self, scheme):
        super(Behavior, self).__init__()

        self.scheme = scheme


class Action(object):
    def __init__(self):
        super(Action, self).__init__()


class Condition(Activatable):
    def __init__(self):
        super(Condition, self).__init__()
