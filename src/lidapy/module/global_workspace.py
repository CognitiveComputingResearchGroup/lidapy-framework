#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Coalition, ConsciousContent


class GlobalWorkspaceModule(FrameworkModule):
    def __init__(self):
        super(GlobalWorkspaceModule, self).__init__("GlobalWorkspaceModule")

    def add_publishers(self):
        pubs = [{"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type()}]
        for pub in pubs:
            super(GlobalWorkspaceModule, self)._add_publisher(pub["topic"], pub["msg_type"])

    def add_subscribers(self):
        subs = [{"topic": "/lida/workspace_coalitions", "msg_type": Coalition.msg_type()}]
        for sub in subs:
            super(GlobalWorkspaceModule, self)._add_subscriber(sub["topic"], sub["msg_type"])

    def advance(self):
        next_coalition = super(GlobalWorkspaceModule, self).get_next_msg("/lida/workspace_coalitions")

        if next_coalition is not None:
            global_broadcast = Coalition()
            global_broadcast.id = next_coalition.id

            super(GlobalWorkspaceModule, self).publish("/lida/global_broadcast", global_broadcast)

if __name__ == '__main__':

    try:
        module = GlobalWorkspaceModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
