#!/usr/bin/env python
'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
from lidapy.util import comm

class FrameworkModule(object):
        
    def __init__(self, name):
        self.name = name

        self._publishers = {}
        self._receivedMsgs = {}

        comm.initialize(self.name)

        self.addPublishers()
        self.addSubscribers()

        return
    
    def _addPublisher(self, topic, msg_type, queue_size=0):
        self._publishers[topic] = comm.getPublisher(topic, msg_type, queue_size = queue_size)
        return
    
    def _addSubscriber(self, topic, msg_type, callback=None, callback_args={}):
        if callback is None:
            callback = self._receiveMsg

        sub_args = {"topic" : topic}
        sub_args.update(callback_args)

        comm.registerSubscriber(topic, msg_type, callback, sub_args)

        self._receivedMsgs[topic] = []

        return

    def _publish(self, topic, msg):
        comm.publishMessage(self._publishers[topic], msg)
        return

    def _receiveMsg(self, msg, args):
        topic = args["topic"]
        if topic is not None:
            msgQueue = self._receivedMsgs[topic]
            if msgQueue is not None:
                msgQueue[topic].append(msg)
        return

    def addPublishers(self):
        pass

    def addSubscribers(self):
        pass

    def getNextMsg(self, topic):
        return self._receivedMsgs[topic].pop()

    def run(self, pubRate):
        comm.run(pubRate)
        return
