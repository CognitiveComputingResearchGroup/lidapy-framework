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


def run(pubRate):
    # set the message publication rate
    rate = rospy.Rate(pubRate)

    while not rospy.is_shutdown():
        rate.sleep()
