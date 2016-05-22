#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class SpatialMemoryModule(FrameworkModule):
    def __init__(self):
        super(SpatialMemoryModule, self).__init__("SpatialMemoryModule", decayable=True, cueable=True)

    # Override this method to add more publishers
    def add_publishers(self):
        super(SpatialMemoryModule, self).add_publisher(built_in_topics["spatial_maps"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(SpatialMemoryModule, self).add_subscriber(built_in_topics["workspace_cues"])
        super(SpatialMemoryModule, self).add_subscriber(built_in_topics["global_broadcast"])

    # Must be overridden
    def advance(self):
        super(SpatialMemoryModule, self).advance()


if __name__ == '__main__':

    try:
        module = SpatialMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
