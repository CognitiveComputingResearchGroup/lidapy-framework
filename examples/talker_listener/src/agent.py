#! /usr/bin/env Python

import random

import lidapy

from lidapy import Config
from lidapy import LIDAModule
from lidapy import Task
from lidapy import Topic

# Shared Talker/Listener Topic
topic = Topic('tl_topic')


# Method invoked by talker
def talk():
    topic.publish(str(random.randint(0, 1000000)))


# Method invoked by listener
def listen():
    lidapy.loginfo('Received message: {}'.format(topic.next_msg))


# Initialize the lidapy framework
lidapy.init(config=Config('../configs/agent.conf'))

LIDAModule('talker', tasks=[Task('talk_task', callback=talk)]).start()
LIDAModule('listener', tasks=[Task('listen_task', callback=listen)]).start()
