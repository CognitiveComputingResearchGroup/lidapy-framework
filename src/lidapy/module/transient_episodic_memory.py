#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class TransientEpisodicMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(TransientEpisodicMemory, self).__init__("TransientEpisodicMemory", decayable=True,
                                                      cueable=True, **kwargs)

    # Override this method to add more publishers
    def add_publishers(self):
        super(TransientEpisodicMemory, self).add_publisher(built_in_topics["episodes"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(TransientEpisodicMemory, self).add_subscriber(built_in_topics["workspace_cues"])
        super(TransientEpisodicMemory, self).add_subscriber(built_in_topics["global_broadcast"])

    # Must be overridden
    def call(self):
        super(TransientEpisodicMemory, self).call()


if __name__ == '__main__':

    try:
        module = TransientEpisodicMemory()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
