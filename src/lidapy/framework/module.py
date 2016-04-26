#!/usr/bin/env python
'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
from lidapy.util import comm, logger


class FrameworkModule(object):
    def __init__(self, name):
        self.name = name

        self._publishers = {}
        self._receivedMsgs = {}

        comm.initialize(self.name)

        self.add_publishers()
        self.add_subscribers()

    def get_param(self, param_name, default):
        decorated_param_name = "/{base_name}/{module_name}/{param_name}".format(base_name='lida', module_name=self.name,
                                                                                param_name=param_name);
        return comm.get_param(decorated_param_name, default)

    def _add_publisher(self, topic, msg_type, queue_size=0):
        logger.info("Adding publisher for topic = {}".format(topic))
        self._publishers[topic] = comm.get_publisher(topic, msg_type, queue_size=queue_size)

    def _add_subscriber(self, topic, msg_type, callback=None, callback_args={}):
        logger.info("Adding subscriber for topic = {}".format(topic))
        if callback is None:
            callback = self._receive_msg

        sub_args = {"topic": topic}
        sub_args.update(callback_args)

        comm.register_subscriber(topic, msg_type, callback, sub_args)
        self._receivedMsgs[topic] = []

    def _publish(self, topic, msg):
        logger.debug("Publishing msg to topic = {} : {}".format(topic, msg))

        comm.publish_message(self._publishers[topic], msg)

    def _receive_msg(self, msg, args):
        topic = args["topic"]
        logger.debug("Receiving message for topic = {} : {}".format(topic, msg))
        if topic is not None:
            msgQueue = self._receivedMsgs[topic]
            if msgQueue is not None:
                msgQueue[topic].append(msg)

    def get_next_msg(self, topic):
        return self._receivedMsgs[topic].pop()

    def advance(self):
        pass

    def run(self):
        while not comm.shutting_down():
            rate_in_hz = self.get_param("rate", 100)
            logger.debug("Current rate is {}".format(rate_in_hz))
            self.advance()
            comm.wait(rate_in_hz)
