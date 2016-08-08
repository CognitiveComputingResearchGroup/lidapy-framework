#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

from lidapy_rosdeps.srv import ccqGetLastNBroadcasts, ccqGetLastNBroadcastsResponse

from collections import deque
from itertools import islice

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "conscious_contents_queue"

# Topics used by this module
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]


class ConsciousContentsQueue(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(ConsciousContentsQueue, self).__init__(name, decayable=True, **kwargs)

        self.max_queue_size = self.config.get_param(name, "max_queue_size", 10)
        self.queue = deque(maxlen=self.max_queue_size)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_subscribers(self):
        super(ConsciousContentsQueue, self).add_subscriber(GLOBAL_BROADCAST_TOPIC)

    # Override this method to add more services
    def add_services(self):
        super(ConsciousContentsQueue, self).add_service("get_last_n_broadcasts",
                                                        ccqGetLastNBroadcasts,
                                                        self.process_last_n_broadcasts_request)

    def get_next_msg(self, topic):
        return super(ConsciousContentsQueue, self).get_next_msg(topic)

    def process_last_n_broadcasts_request(self, request):

        queue_size = len(self.queue)
        if request.last_n > queue_size:
            request.last_n = queue_size

        response = ccqGetLastNBroadcastsResponse()
        response.items = list(islice(self.queue, queue_size - request.last_n, queue_size))

        return response

    def call(self):
        broadcast = self.get_next_msg(GLOBAL_BROADCAST_TOPIC)

        if broadcast is not None:
            self.queue.append(broadcast)


if __name__ == '__main__':

    try:
        module = ConsciousContentsQueue()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
