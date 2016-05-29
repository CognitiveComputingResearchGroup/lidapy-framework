#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class SpatialMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(SpatialMemory, self).__init__("SpatialMemory", decayable=True, cueable=True, **kwargs)

    # Override this method to add more publishers
    def add_publishers(self):
        super(SpatialMemory, self).add_publisher(built_in_topics["spatial_maps"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(SpatialMemory, self).add_subscriber(built_in_topics["workspace_cues"])
        super(SpatialMemory, self).add_subscriber(built_in_topics["global_broadcast"])

    # Must be overridden
    def advance(self):
        super(SpatialMemory, self).advance()


if __name__ == '__main__':

    try:
        module = SpatialMemory()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
