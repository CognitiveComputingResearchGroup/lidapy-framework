#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Behavior, ConsciousContent
# TODO: Replace this with LIDA msgs
from std_msgs.msg import String


class SensoryMotorMemoryModule(FrameworkModule):
    def __init__(self):
        super(SensoryMotorMemoryModule, self).__init__("SensoryMotorMemoryModule")
        return

    def add_publishers(self):
        #pubs = [{"topic": "/lida/motor_commands", "msg_type": String}]
        pubs = []
        for pub in pubs:
            super(SensoryMotorMemoryModule, self)._add_publisher(pub["topic"], pub["msg_type"])

        return

    def add_subscribers(self):
        subs = [{"topic": "/lida/selected_behaviors", "msg_type": Behavior.msg_type()},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type()},]
              #  {"topic": "/lida/dorsal_stream", "msg_type": String}]
        for sub in subs:
            super(SensoryMotorMemoryModule, self)._add_subscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = SensoryMotorMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass
