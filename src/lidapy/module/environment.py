#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
# TODO: Replace this with actual messages
from std_msgs.msg import String


class EnvironmentModule(FrameworkModule):
    def __init__(self):
        super(EnvironmentModule, self).__init__("EnvironmentModule")

    def add_publishers(self):
        # TODO: Add actuator commands
        # pubs = [{"topic": "", "msg_type" : }]
        pubs = []
        for pub in pubs:
            super(EnvironmentModule, self)._add_publisher(pub["topic"], pub["msg_type"])

    def add_subscribers(self):
        # TODO: Add sensor input topic
        # subs = [{"topic": "/lida/environment", "msg_type" : String}]
        # subs = [{"topic": """, "msg_type" : }]
        subs = []
        for sub in subs:
            super(EnvironmentModule, self)._add_subscriber(sub["topic"], sub["msg_type"])


if __name__ == '__main__':

    try:
        module = EnvironmentModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass
