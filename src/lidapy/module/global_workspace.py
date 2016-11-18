from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "global_workspace"

# Topics used by this module
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")
WORKSPACE_COALITIONS_TOPIC = FrameworkTopic("workspace_coalitions")


class GlobalWorkspace(FrameworkModule):
    def __init__(self):
        super(GlobalWorkspace, self).__init__()

        self.add_publishers([GLOBAL_BROADCAST_TOPIC])
        self.add_subscribers([WORKSPACE_COALITIONS_TOPIC])

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def call(self):
        workspace_coalitions = WORKSPACE_COALITIONS_TOPIC.subscriber.get_next_msg()

        if workspace_coalitions is not None:
            global_broadcast = workspace_coalitions

            GLOBAL_BROADCAST_TOPIC.publisher.publish(global_broadcast)
