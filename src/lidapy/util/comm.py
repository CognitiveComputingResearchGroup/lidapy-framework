#!/usr/bin/env python
'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
import rospy


def initialize(name):
    rospy.init_node(name)


def get_publisher(topic, msg_type, queue_size=0):
    return rospy.Publisher(topic, msg_type, queue_size=queue_size)


def register_subscriber(topic, msg_type, callback, callback_args=[]):
    rospy.Subscriber(topic, msg_type, callback=callback, callback_args=callback_args)


def publish_message(publisher, msg):
    publisher.publish(msg.serializable_msg)


def get_param(param_name, default=None):
    return rospy.get_param(param_name, default)


def shutting_down():
    return rospy.is_shutdown()


def wait(rate):
    waiter = rospy.Rate(rate)
    waiter.sleep()
