#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Behavior, ConsciousContent


class ActionSelectionModule(FrameworkModule):
    def __init__(self):
        super(ActionSelectionModule, self).__init__("ActionSelectionModule")
        return

    def add_publishers(self):
        pubs = [{"topic": "/lida/selected_behaviors", "msg_type": Behavior.msg_type()}]

        for pub in pubs:
            super(ActionSelectionModule, self)._add_publisher(pub["topic"], pub["msg_type"])

        return

    def add_subscribers(self):
        subs = [{"topic": "/lida/candidate_behaviors", "msg_type": Behavior.msg_type()},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type()}]
        for sub in subs:
            super(ActionSelectionModule, self)._add_subscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = ActionSelectionModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass
