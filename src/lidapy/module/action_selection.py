from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class ActionSelection(FrameworkModule):
    def __init__(self, **kwargs):
        super(ActionSelection, self).__init__("action_selection", decayable=True, **kwargs)

    # Override this method to add more publishers
    def add_publishers(self):
        super(ActionSelection, self).add_publisher(built_in_topics["selected_behaviors"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(ActionSelection, self).add_subscriber(built_in_topics["candidate_behaviors"])
        super(ActionSelection, self).add_subscriber(built_in_topics["global_broadcast"])

    # Should be overridden
    def call(self):
        super(ActionSelection, self).call()

        candidate_behaviors = super(ActionSelection, self).get_next_msg("candidate_behaviors")

        if candidate_behaviors is not None:
            selected_behaviors = candidate_behaviors

            self.publishers["selected_behaviors"].publish(selected_behaviors)
