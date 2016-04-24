#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from std_msgs.msg import String


class PerceptualAssociativeMemoryModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "PerceptualAssociativeMemoryModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/percepts", "msg_type" : String}]
        for pub in pubs:
            super(PerceptualAssociativeMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/sensory_memory", "msg_type" : String},
                {"topic": "/lida/workspace_cue", "msg_type": String}]

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

