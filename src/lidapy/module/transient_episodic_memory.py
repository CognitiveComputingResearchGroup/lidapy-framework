#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from std_msgs.msg import String


class TransientEpisodicMemoryModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "TransientEpisodicMemoryModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/episodic_memory", "msg_type" : String}]
        for pub in pubs:
            super(TransientEpisodicMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/workspace_cue", "msg_type" : String}]
        for sub in subs:
            super(TransientEpisodicMemoryModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = TransientEpisodicMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

