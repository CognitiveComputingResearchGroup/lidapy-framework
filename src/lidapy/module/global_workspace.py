#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lida.msg import Coalition, ConsciousContent


class GlobalWorkspaceModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "GlobalWorkspaceModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/global_broadcast", "msg_type" : ConsciousContent}]
        for pub in pubs:
            super(GlobalWorkspaceModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/workspace_coalitions", "msg_type" : Coalition}]
        for sub in subs:
            super(GlobalWorkspaceModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = GlobalWorkspaceModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

