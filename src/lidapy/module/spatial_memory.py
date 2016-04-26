#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import ConsciousContent, Cue, SpatialMap


class SpatialMemoryModule(FrameworkModule):
    def __init__(self):
        super(SpatialMemoryModule, self).__init__("SpatialMemoryModule")

    def add_publishers(self):
        pubs = [{"topic": "/lida/spatial_maps", "msg_type": SpatialMap.msg_type()}]
        for pub in pubs:
            super(SpatialMemoryModule, self)._add_publisher(pub["topic"], pub["msg_type"])

    def add_subscribers(self):
        subs = [{"topic": "/lida/workspace_cues", "msg_type": Cue.msg_type()},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type()}]
        for sub in subs:
            super(SpatialMemoryModule, self)._add_subscriber(sub["topic"], sub["msg_type"])


if __name__ == '__main__':

    try:
        module = SpatialMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
