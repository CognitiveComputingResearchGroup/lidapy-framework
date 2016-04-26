#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Behavior, ConsciousContent


class ProceduralMemoryModule(FrameworkModule):
    def __init__(self):
        super(ProceduralMemoryModule, self).__init__("ProceduralMemoryModule")

    def add_publishers(self):
        pubs = [{"topic": "/lida/candidate_behaviors", "msg_type": Behavior.msg_type()}]
        for pub in pubs:
            super(ProceduralMemoryModule, self)._add_publisher(pub["topic"], pub["msg_type"])

    def add_subscribers(self):
        subs = [{"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type()}]
        for sub in subs:
            super(ProceduralMemoryModule, self)._add_subscriber(sub["topic"], sub["msg_type"])


if __name__ == '__main__':

    try:
        module = ProceduralMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
