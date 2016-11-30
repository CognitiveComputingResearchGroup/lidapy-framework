from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.shared import Activatable, CognitiveContent

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


class Behavior(Activatable):
    def __init__(self, unique_id, action=None, context_conditions=set(), adding_list=set(), deleting_list=set()):
        super(Behavior, self).__init__()

        self.unique_id = unique_id
        self.action = action
        self.context_conditions = set(context_conditions)
        self.adding_list = set(adding_list)
        self.deleting_list = set(deleting_list)


class Action(Activatable):
    def __init__(self, unique_id):
        super(Action, self).__init__()

        self.unique_id = unique_id


class Condition(Activatable):
    def __init__(self, unique_id, predicate):
        super(Condition, self).__init__()

        self.unique_id = unique_id
        self.predicate = predicate

    def check(self, *args, **kwargs):
        return self.predicate(*args, **kwargs)
