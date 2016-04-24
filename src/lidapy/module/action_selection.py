#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule
from lida.msg import Behavior, ConsciousContent


class ActionSelectionModule(FrameworkModule):

    def __init__(self):
        super(ActionSelectionModule, self).__init__("ActionSelectionModule")
        return

    def addPublishers(self):
        pubs = [{"topic": "/lida/selected_behaviors", "msg_type" : Behavior}]
        for pub in pubs:
            super(ActionSelectionModule, self)._addPublisher(pub["topic"], pub["msg_type"])

        return

    def addSubscribers(self):
        subs = [{"topic": "/lida/candidate_behaviors", "msg_type" : Behavior},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent}]
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
