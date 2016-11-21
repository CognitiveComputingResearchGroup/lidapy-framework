from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic


# Topics used by this module
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")
WORKSPACE_COALITIONS = FrameworkTopic("workspace_coalitions")


class GlobalWorkspace(FrameworkModule):
    def __init__(self):
        super(GlobalWorkspace, self).__init__()

        self.add_publishers([GLOBAL_BROADCAST])
        self.add_subscribers([WORKSPACE_COALITIONS])

    @classmethod
    def get_module_name(cls):
        return "global_workspace"

    def call(self):
        workspace_coalitions = WORKSPACE_COALITIONS.subscriber.get_next_msg()

        if workspace_coalitions is not None:
            global_broadcast = workspace_coalitions

            GLOBAL_BROADCAST.publisher.publish(global_broadcast)
