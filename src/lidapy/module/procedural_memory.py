#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lida.msg import Behavior, ConsciousContent


class ProceduralMemoryModule(FrameworkModule):

    def __init__(self):
        super(ProceduralMemoryModule, self).__init__("ProceduralMemoryModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/candidate_behaviors", "msg_type" : Behavior}]
        for pub in pubs:
            super(ProceduralMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/global_broadcast", "msg_type" : ConsciousContent}]
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

