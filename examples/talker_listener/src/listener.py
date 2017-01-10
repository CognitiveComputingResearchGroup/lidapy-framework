#! /usr/bin/env Python

import time
import lidapy

topic = lidapy.Topic('tl_topic')

# Initialize the lidapy framework
lidapy.init(config=lidapy.Config('../configs/agent.conf'), process_name='listener')

while True:
    lidapy.loginfo('Received message: {}'.format(topic.next_msg))
    time.sleep(1)
