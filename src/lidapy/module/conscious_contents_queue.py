#!/usr/bin/env python

from collections import deque
from itertools import islice

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics, MsgSerializer
from lidapy_rosdeps.srv import GenericService

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "conscious_contents_queue"

# Topics used by this module
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]


class ConsciousContentsQueue(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(ConsciousContentsQueue, self).__init__(name, **kwargs)

        self.max_queue_size = self.config.get_param(name, "max_queue_size", 10)
        self.queue = deque(maxlen=self.max_queue_size)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_subscribers(self):
        self.add_subscriber(GLOBAL_BROADCAST_TOPIC)

    # Override this method to add more services
    def add_services(self):
        self.add_service("get_last_n_broadcasts", GenericService, self.process_last_n_broadcasts_request)

    def process_last_n_broadcasts_request(self, raw_request):

        request = MsgSerializer.deserialize(raw_request)
        queue_size = len(self.queue)
        if request.n > queue_size:
            request.n = queue_size

        response = CcqGetLastNBroadcastsResponse()
        response.last_n_broadcasts = list(islice(self.queue, queue_size - request.last_n, queue_size))

        return response

    def call(self):
        broadcast = self.get_next_msg(GLOBAL_BROADCAST_TOPIC)

        if broadcast is not None:
            self.queue.append(broadcast)


class CcqGetLastNBroadcastsRequest(object):
    def __init__(self, n):
        self.n = n


class CcqGetLastNBroadcastsResponse(object):
    def __init__(self, last_n_broadcasts):
        self.last_n_broadcasts = last_n_broadcasts


if __name__ == '__main__':

    try:
        module = ConsciousContentsQueue()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
