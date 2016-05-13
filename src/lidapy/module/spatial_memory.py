#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class SpatialMemoryModule(FrameworkModule):
    def __init__(self):
        super(SpatialMemoryModule, self).__init__("SpatialMemoryModule")

    # Override this method to add more publishers
    def add_publishers(self):
        super(SpatialMemoryModule, self).add_publisher(built_in_topics["/lida/spatial_maps"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(SpatialMemoryModule, self).add_subscriber(built_in_topics["/lida/workspace_cues"])
        super(SpatialMemoryModule, self).add_subscriber(built_in_topics["/lida/global_broadcast"])


if __name__ == '__main__':

    try:
        module = SpatialMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
