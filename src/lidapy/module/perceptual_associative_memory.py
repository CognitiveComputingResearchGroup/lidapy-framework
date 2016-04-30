#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import ConsciousContent, Cue, Feature, Percept


class PerceptualAssociativeMemoryModule(FrameworkModule):
    def __init__(self):
        super(PerceptualAssociativeMemoryModule, self).__init__("PerceptualAssociativeMemoryModule")

    def add_publishers(self):
        pubs = [{"topic": "/lida/percepts", "msg_type": Percept.msg_type()}]
        for pub in pubs:
            super(PerceptualAssociativeMemoryModule, self)._add_publisher(pub["topic"], pub["msg_type"])

    def add_subscribers(self):
        subs = [{"topic": "/lida/detected_features", "msg_type": Feature.msg_type()},
                {"topic": "/lida/workspace_cues", "msg_type": Cue.msg_type()},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type()}]

        for sub in subs:
            super(PerceptualAssociativeMemoryModule, self)._add_subscriber(sub["topic"], sub["msg_type"])

    def advance(self):

        next_feature = super(PerceptualAssociativeMemoryModule, self).get_next_msg("/lida/detected_features")

        if next_feature is not None:
            percept = Percept()
            percept.id = next_feature.id

            super(PerceptualAssociativeMemoryModule, self).publish("/lida/percepts", percept)


if __name__ == '__main__':

    try:
        module = PerceptualAssociativeMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
