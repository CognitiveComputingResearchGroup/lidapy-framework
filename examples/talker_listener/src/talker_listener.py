#! /usr/bin/env Python

import random

from lidapy import Config
from lidapy import LIDAThread
from lidapy import Topic
from lidapy import init, loginfo

# Shared Talker/Listener Topic
topic = Topic('tl_topic')


# Method invoked by talker
def talk():
    topic.publish(str(random.randint(0, 1000000)))


# Method invoked by listener
def listen():
    loginfo('Received message: {}'.format(topic.next_msg))


# Initialize the lidapy framework
init(config=Config('../configs/agent.conf'), module_name='talker_listener')

LIDAThread(name='talker', callback=talk).start()
LIDAThread(name='listener', callback=listen).start()
