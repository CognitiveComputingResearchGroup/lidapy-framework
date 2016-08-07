from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class GlobalWorkspace(FrameworkModule):
    def __init__(self, **kwargs):
        super(GlobalWorkspace, self).__init__("global_workspace", decayable=True, **kwargs)

    # Override this method to add more publishers
    def add_publishers(self):
        super(GlobalWorkspace, self).add_publisher(built_in_topics["global_broadcast"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(GlobalWorkspace, self).add_subscriber(built_in_topics["workspace_coalitions"])

    # Should be overridden
    def call(self):
        super(GlobalWorkspace, self).call()

        workspace_coalitions = super(GlobalWorkspace, self).get_next_msg("workspace_coalitions")

        if workspace_coalitions is not None:
            global_broadcast = workspace_coalitions

            self.publishers["global_broadcast"].publish(global_broadcast)
