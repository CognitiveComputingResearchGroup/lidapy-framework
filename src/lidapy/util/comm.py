'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
import rospy

def initialize(name):
    rospy.init_node(name)
    return

def getPublisher(topic, msg_type, queue_size=0):
    return rospy.Publisher(topic, msg_type, queue_size=queue_size)

def registerSubscriber(topic, msg_type, callback, callback_args=[]):
    rospy.Subscriber(topic, msg_type, callback=callback, callback_args=callback_args)
    return

def publishMessage(publisher, msg):
    publisher.publish(msg)
    return