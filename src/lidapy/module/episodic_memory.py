from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class EpisodicMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(EpisodicMemory, self).__init__("EpisodicMemory", decayable=True, cueable=True, **kwargs)

    # Override this method to add more publishers
    def add_publishers(self):
        super(EpisodicMemory, self).add_publisher(built_in_topics["episodes"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(EpisodicMemory, self).add_subscriber(built_in_topics["workspace_cues"])
        super(EpisodicMemory, self).add_subscriber(built_in_topics["global_broadcast"])

    # Must be overridden
    def call(self):
        super(EpisodicMemory, self).call()
