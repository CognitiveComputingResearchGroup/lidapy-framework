#!/usr/bin/env python
'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
import rospy


def initialize(name):
    rospy.init_node(name, log_level=rospy.DEBUG)


def get_publisher(topic, msg_type, queue_size=0):
    return rospy.Publisher(topic, msg_type, queue_size=queue_size)


def register_subscriber(topic, msg_type, callback, callback_args=[]):
    return rospy.Subscriber(topic, msg_type, callback=callback, callback_args=callback_args)


def publish_message(publisher, msg):
    publisher.publish(msg.serialize())


def get_service(service_name, service_class, callback):
    return rospy.Service(service_name, service_class, callback)


def get_service_proxy(service_name, service_class):
    return rospy.ServiceProxy(service_name, service_class)


def shutting_down():
    return rospy.is_shutdown()


def wait(rate):
    waiter = rospy.Rate(rate)
    waiter.sleep()


class ParameterService(object):

    def get_param(self, param_type, param_name, default_value=None):
        fully_qualified_name = self.get_fully_qualified_param_name(param_type, param_name)
        return rospy.get_param(fully_qualified_name, default_value)

    def set_param(self, param_type, param_name, param_value):
        fully_qualified_name = self.get_fully_qualified_param_name(param_type, param_name)
        return rospy.set_param(fully_qualified_name, param_value)

    def get_fully_qualified_param_name(self, param_type, param_name):
        return "/".join(["lida", param_type, param_name])
