#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class SensoryMemoryModule(FrameworkModule):
    def __init__(self):
        super(SensoryMemoryModule, self).__init__("SensoryMemoryModule", decayable=True)

        self.add_publishers()
        self.add_subscribers()

    # Override this method to add more publishers
    def add_publishers(self):
        super(SensoryMemoryModule, self).add_publisher(built_in_topics["dorsal_stream"])
        super(SensoryMemoryModule, self).add_publisher(built_in_topics["ventral_stream"])
        super(SensoryMemoryModule, self).add_publisher(built_in_topics["detected_features"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(SensoryMemoryModule, self).add_subscriber(built_in_topics["global_broadcast"])

    # Must be overridden
    def advance(self):
        super(SensoryMemoryModule, self).advance()


if __name__ == '__main__':

    try:
        module = SensoryMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
