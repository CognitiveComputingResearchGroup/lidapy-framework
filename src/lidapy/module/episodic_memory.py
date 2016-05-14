#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class EpisodicMemoryModule(FrameworkModule):
    def __init__(self):
        super(EpisodicMemoryModule, self).__init__("EpisodicMemoryModule")

    # Override this method to add more publishers
    def add_publishers(self):
        super(EpisodicMemoryModule, self).add_publisher(built_in_topics["/lida/episodes"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(EpisodicMemoryModule, self).add_subscriber(built_in_topics["/lida/workspace_cues"])
        super(EpisodicMemoryModule, self).add_subscriber(built_in_topics["/lida/global_broadcast"])


if __name__ == '__main__':

    try:
        module = EpisodicMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
