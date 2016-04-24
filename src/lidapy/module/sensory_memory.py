#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from std_msgs.msg import String


class SensoryMemoryModule(FrameworkModule):

    def __init__(self):
        super(SensoryMemoryModule, self).__init__("SensoryMemoryModule")

        self.addPublishers()
        self.addSubscribers()

        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/sensory_memory", "msg_type" : String}]
        for pub in pubs:
            super(SensoryMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/environment", "msg_type" : String}]
        for sub in subs:
            super(SensoryMemoryModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = SensoryMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass
