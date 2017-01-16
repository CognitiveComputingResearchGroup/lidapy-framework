#! /usr/bin/env python

import random
from sys import argv

from lidapy import Config
from lidapy import LIDAThread
from lidapy import Topic
from lidapy import init, loginfo

# Shared Talker/Listener Topic
topic = Topic('tl_topic')


# Method invoked by talker
def talk():
    topic.send(str(random.randint(0, 1000000)))


# Method invoked by listener
def listen():
    loginfo('Received message: {}'.format(topic.receive()))


# Initialize the lidapy framework
init(config=Config(argv[1]), process_name='talker_listener')

LIDAThread(name='talker', callback=talk).start()
LIDAThread(name='listener', callback=listen).start()
