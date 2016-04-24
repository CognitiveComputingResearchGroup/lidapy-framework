#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from std_msgs.msg import String


class ActionSelectionModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "ActionSelectionModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/action_selection", "msg_type" : String}]
        for pub in pubs:
            super(ActionSelectionModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/procedural_memory", "msg_type" : String},
                {"topic": "/lida/global_broadcast", "msg_type": String}]
        for sub in subs:
            super(ActionSelectionModule, self)._addSubscriber(sub["topic"], sub["msg_type"])

        return


if __name__ == '__main__':

    try:
        module = ActionSelectionModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass
