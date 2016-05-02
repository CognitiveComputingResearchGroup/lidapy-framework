#!/usr/bin/env python
'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
from lidapy.util import comm, logger
from lidapy.framework.agent import AgentConfig

class FrameworkModule(object):
    def __init__(self, module_name):
        self.module_name = module_name

        comm.initialize(self.module_name)

        self._publishers = {}
        self._received_msgs = {}

        self._config = AgentConfig()

        self.add_publishers()
        self.add_subscribers()

    def get_param(self, param_name, default_value=None):
        param_value = self._config.get_param(self.module_name, param_name, default_value)
        logger.info("{} = {}".format(param_name, param_value))
        return param_value

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
        self._received_msgs[topic] = []

    def _receive_msg(self, msg, args):
        topic = args["topic"]
        logger.debug("Receiving message for topic = {} : {}".format(topic, msg))
        if topic is not None:
            msg_queue = self._received_msgs[topic]
            msg_queue.append(msg)

    def get_next_msg(self, topic):
        msg_queue = self._received_msgs[topic]

        next_msg = None
        if len(msg_queue) > 0:
            next_msg = self._received_msgs[topic].pop()

        return next_msg

    def publish(self, topic, msg):
        logger.debug("Publishing msg to topic = {} : {}".format(topic, msg))

        comm.publish_message(self._publishers[topic], msg)

    def advance(self):
        pass

    def run(self):
        while not comm.shutting_down():
            rate_in_hz = self.get_param("rate_in_hz", 100)
            self.advance()
            comm.wait(rate_in_hz)
