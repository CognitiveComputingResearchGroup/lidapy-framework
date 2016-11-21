#!/usr/bin/env python

from collections import deque
from itertools import islice

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic
from lidapy.util.comm import MsgUtils
from lidapy_rosdeps.srv import GenericService


# Topics used by this module
GLOBAL_BROADCAST = FrameworkTopic("global_broadcast")


class ConsciousContentsQueue(FrameworkModule):
    def __init__(self):
        super(ConsciousContentsQueue, self).__init__()

        self.max_queue_size = self.config.get_param(self.name, "max_queue_size", 10)
        self.queue = deque(maxlen=self.max_queue_size)

        self.add_subscribers([GLOBAL_BROADCAST])

        self.add_service("get_last_n_broadcasts", GenericService, self.process_last_n_broadcasts_request)

    @classmethod
    def get_module_name(cls):
        return "conscious_contents_queue"

    def process_last_n_broadcasts_request(self, raw_request):

        request = MsgUtils.deserialize(raw_request)
        queue_size = len(self.queue)
        if request.n > queue_size:
            request.n = queue_size

        response = CcqGetLastNBroadcastsResponse()
        response.last_n_broadcasts = list(islice(self.queue, queue_size - request.last_n, queue_size))

        return response

    def call(self):
        broadcast = GLOBAL_BROADCAST.subscriber.get_next_msg()

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
