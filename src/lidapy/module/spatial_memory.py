#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lida.msg import ConsciousContent, Cue, SpatialMap


class SpatialMemoryModule(FrameworkModule):

    def __init__(self):
        super(SpatialMemoryModule, self).__init__("SpatialMemoryModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/spatial_maps", "msg_type" : SpatialMap}]
        for pub in pubs:
            super(SpatialMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/workspace_cues", "msg_type" : Cue},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent}]
        for sub in subs:
            super(SpatialMemoryModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = SpatialMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

