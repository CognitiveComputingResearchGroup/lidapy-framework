#! /usr/bin/env Python

import random
import time

from lidapy import Config
from lidapy import Topic
from lidapy import init, loginfo

topic = Topic('tl_topic')

# Initialize the lidapy framework
init(config=Config('../configs/agent.conf'), module_name='talker')

while True:
    msg = str(random.randint(1, 10))
    loginfo('Sending message: {}'.format(msg))
    topic.publish(msg)
    time.sleep(1)
