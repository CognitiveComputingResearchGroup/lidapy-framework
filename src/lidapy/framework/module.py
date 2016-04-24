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

        comm.initialize(self.name)

        return
    
    def _addPublisher(self, topic, msg_type, queue_size=0):
        self._publishers[topic] = comm.getPublisher(topic, msg_type, queue_size = queue_size)
        return
    
    def _addSubscriber(self, topic, msg_type, callback, callback_args=[]):
        comm.registerSubscriber(topic, msg_type, callback, callback_args)
        return

    def _publish(self, topic, msg):
        comm.publishMessage(self._publishers[topic], msg)
        return

    def run(self, pubRate):
        comm.run(pubRate)
        return
