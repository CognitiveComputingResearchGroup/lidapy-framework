#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Feature
# TODO: Replace this with LIDA messages
from std_msgs.msg import String


class SensoryMemoryModule(FrameworkModule):
    def __init__(self):
        super(SensoryMemoryModule, self).__init__("SensoryMemoryModule")

        self.add_publishers()
        self.add_subscribers()

    def add_publishers(self):
        #{"topic": "/lida/dorsal_stream", "msg_type": String},
        #{"topic": "/lida/ventral_stream", "msg_type": String},

        pubs = [{"topic": "/lida/detected_features", "msg_type": Feature.msg_type()},]
        for pub in pubs:
            super(SensoryMemoryModule, self)._add_publisher(pub["topic"], pub["msg_type"])

    def add_subscribers(self):
        #subs = [{"topic": "/lida/environment", "msg_type": String}]
        subs = []
        for sub in subs:
            super(SensoryMemoryModule, self)._add_subscriber(sub["topic"], sub["msg_type"])


if __name__ == '__main__':

    try:
        module = SensoryMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
