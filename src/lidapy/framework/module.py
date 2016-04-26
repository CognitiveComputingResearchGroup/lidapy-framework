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

        self.add_publishers()
        self.add_subscribers()

    def _add_publisher(self, topic, msg_type, queue_size=0):
        self._publishers[topic] = comm.get_publisher(topic, msg_type, queue_size=queue_size)

    def _add_subscriber(self, topic, msg_type, callback=None, callback_args={}):
        if callback is None:
            callback = self._receive_msg

        sub_args = {"topic": topic}
        sub_args.update(callback_args)

        comm.register_subscriber(topic, msg_type, callback, sub_args)

        self._receivedMsgs[topic] = []

    def _publish(self, topic, msg):
        comm.publish_message(self._publishers[topic], msg)

    def _receive_msg(self, msg, args):
        topic = args["topic"]
        if topic is not None:
            msgQueue = self._receivedMsgs[topic]
            if msgQueue is not None:
                msgQueue[topic].append(msg)

    def add_publishers(self):
        pass

    def add_subscribers(self):
        pass

    def get_next_msg(self, topic):
        return self._receivedMsgs[topic].pop()

    def run(self, pubRate):
        comm.run(pubRate)
