#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from std_msgs.msg import String


class ProceduralMemoryModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "ProceduralMemoryModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/procedural_memory", "msg_type" : String}]
        for pub in pubs:
            super(ProceduralMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/global_broadcast", "msg_type" : String}]
        for sub in subs:
            super(ProceduralMemoryModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = ProceduralMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

