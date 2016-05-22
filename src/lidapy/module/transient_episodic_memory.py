#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class TransientEpisodicMemoryModule(FrameworkModule):
    def __init__(self):
        super(TransientEpisodicMemoryModule, self).__init__("TransientEpisodicMemoryModule")

    # Override this method to add more publishers
    def add_publishers(self):
        super(TransientEpisodicMemoryModule, self).add_publisher(built_in_topics["episodes"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(TransientEpisodicMemoryModule, self).add_subscriber(built_in_topics["workspace_cues"])
        super(TransientEpisodicMemoryModule, self).add_subscriber(built_in_topics["global_broadcast"])

    # Must be overridden
    def advance(self):
        super(TransientEpisodicMemoryModule, self).advance()


if __name__ == '__main__':

    try:
        module = TransientEpisodicMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
