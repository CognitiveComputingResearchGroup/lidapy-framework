from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "global_workspace"

# Topics used by this module
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]
WORKSPACE_COALITIONS_TOPIC = built_in_topics["workspace_coalitions"]


class GlobalWorkspace(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(GlobalWorkspace, self).__init__(name, **kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        self.add_publisher(GLOBAL_BROADCAST_TOPIC)

    def add_subscribers(self):
        self.add_subscriber(WORKSPACE_COALITIONS_TOPIC)

    def call(self):
        workspace_coalitions = self.get_next_msg(WORKSPACE_COALITIONS_TOPIC)

        if workspace_coalitions is not None:
            global_broadcast = workspace_coalitions

            self.publish(GLOBAL_BROADCAST_TOPIC, global_broadcast)
