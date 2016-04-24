#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lida.msg import Behavior, ConsciousContent
#TODO: Replace this with LIDA msgs
from std_msgs.msg import String


class SensoryMotorMemoryModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "SensoryMotorMemoryModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/motor_commands", "msg_type" : String}]
        for pub in pubs:
            super(SensoryMotorMemoryModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/selected_behaviors", "msg_type" : Behavior},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent},
                {"topic": "/lida/dorsal_stream", "msg_type": String}]
        for sub in subs:
            super(SensoryMotorMemoryModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = SensoryMotorMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

