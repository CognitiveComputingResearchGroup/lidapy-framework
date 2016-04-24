#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
# TODO: Replace with LIDA msgs
from std_msgs.msg import String
from lida.msg import Coalition, ConsciousContent, Cue, Episode, Percept, SpatialMap


class WorkspaceModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "WorkspaceModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/workspace_coalitions", "msg_type" : Coalition},
                {"topic": "/lida/workspace_cues", "msg_type": Cue}]
        for pub in pubs:
            super(WorkspaceModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/ventral_stream", "msg_type" : String},
                {"topic": "/lida/percepts", "msg_type": Percept},
                {"topic": "/lida/spatial_maps", "msg_type": SpatialMap},
                {"topic": "/lida/episodes", "msg_type": Episode},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent},]
        for sub in subs:
            super(WorkspaceModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = WorkspaceModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

