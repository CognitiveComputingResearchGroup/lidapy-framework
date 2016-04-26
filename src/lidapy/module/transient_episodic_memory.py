#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import ConsciousContent, Cue, Episode


class TransientEpisodicMemoryModule(FrameworkModule):
    def __init__(self):
        super(TransientEpisodicMemoryModule, self).__init__("TransientEpisodicMemoryModule")

    def add_publishers(self):
        pubs = [{"topic": "/lida/episodes", "msg_type": Episode.msg_type()}]
        for pub in pubs:
            super(TransientEpisodicMemoryModule, self)._add_publisher(pub["topic"], pub["msg_type"])

    def add_subscribers(self):
        subs = [{"topic": "/lida/workspace_cues", "msg_type": Cue.msg_type()},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type()}]
        for sub in subs:
            super(TransientEpisodicMemoryModule, self)._add_subscriber(sub["topic"], sub["msg_type"])


if __name__ == '__main__':

    try:
        module = TransientEpisodicMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
