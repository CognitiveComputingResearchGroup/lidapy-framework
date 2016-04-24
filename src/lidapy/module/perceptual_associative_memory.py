#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lida.msg import ConsciousContent, Cue, Feature, Percept


class PerceptualAssociativeMemoryModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "PerceptualAssociativeMemoryModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/percepts", "msg_type" : Percept}]
        for pub in pubs:
            super(PerceptualAssociativeMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/detected_features", "msg_type" : Feature},
                {"topic": "/lida/workspace_cues", "msg_type": Cue},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent}]

        for sub in subs:
            super(PerceptualAssociativeMemoryModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = PerceptualAssociativeMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

