#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
# TODO: Replace with LIDA msgs
from std_msgs.msg import String
from lidapy.framework.msg import Coalition, ConsciousContent, Cue, Episode, Percept, SpatialMap


class WorkspaceModule(FrameworkModule):
    def __init__(self):
        super(WorkspaceModule, self).__init__("WorkspaceModule")
        return

    def add_publishers(self):
        pubs = [{"topic": "/lida/workspace_coalitions", "msg_type": Coalition.msg_type()},
                {"topic": "/lida/workspace_cues", "msg_type": Cue.msg_type()}]
        for pub in pubs:
            super(WorkspaceModule, self)._add_publisher(pub["topic"], pub["msg_type"])

        return

    def add_subscribers(self):
        # {"topic": "/lida/ventral_stream", "msg_type": String},
        subs = [{"topic": "/lida/percepts", "msg_type": Percept.msg_type()},
                {"topic": "/lida/spatial_maps", "msg_type": SpatialMap.msg_type()},
                {"topic": "/lida/episodes", "msg_type": Episode.msg_type()},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type()},]
        for sub in subs:
            super(WorkspaceModule, self)._add_subscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = WorkspaceModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass
