#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lida.msg import ConsciousContent, Cue, Episode


class EpisodicMemoryModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "EpisodicMemoryModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/episodes", "msg_type" : Episode}]
        for pub in pubs:
            super(EpisodicMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/workspace_cues", "msg_type" : Cue},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent}]
        for sub in subs:
            super(EpisodicMemoryModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = EpisodicMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

