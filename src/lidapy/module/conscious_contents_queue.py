#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics, FrameworkMsg
from lidapy.framework.service import FrameworkService

# TODO: This is a ROS specific detail.  Need to figure out how to hide this!
from lida.srv import ccqGetLastNBroadcasts
from collections import deque
from itertools import islice


class ConsciousContentsQueue(FrameworkModule):
    def __init__(self):
        super(ConsciousContentsQueue, self).__init__("ConsciousContentsQueue")

        self.max_queue_size = self.config.get_param("ConsciousContentsQueue",
                                                    "max_queue_size", 10)

        self.queue = deque(self.max_queue_size)
        self.service = FrameworkService("ccqGetLastNBroadcasts",
                                        ccqGetLastNBroadcasts,
                                        self.process_last_n_broadcasts_request)

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(ConsciousContentsQueue, self).add_subscriber(built_in_topics["/lida/global_broadcast"])

    def process_last_n_broadcasts_request(self, request):

        queue_size = len(self.queue)
        if request.last_n > queue_size:
            request.last_n = queue_size

        response = ccqGetLastNBroadcastsResponse()
        response.items = islice(self.queue, queue_size - request.last_n, queue_size)

        return response

    def advance(self):
        self.logger.debug("Inside advance")

        broadcast = super(ConsciousContentsQueue, self).get_next_msg("/lida/global_broadcast")

        if broadcast is not None:
            self.queue.append(broadcast)


class ccqGetLastNBroadcastsRequest(FrameworkMsg):
    def __init__(self):
        super(ccqGetLastNBroadcastsRequest, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.srv import ccqGetLastNBroadcastsRequest as _ccqGetLastNBroadcastsRequest
            return _ccqGetLastNBroadcastsRequest
        except ImportError:
            return ccqGetLastNBroadcastsRequest

    def _create_serializable_msg(self):
        return ccqGetLastNBroadcastsRequest.msg_type()()


class ccqGetLastNBroadcastsResponse(FrameworkMsg):
    def __init__(self):
        super(ccqGetLastNBroadcastsResponse, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.srv import ccqGetLastNBroadcastsResponse as _ccqGetLastNBroadcastsResponse
            return ccqGetLastNBroadcastsResponse
        except ImportError:
            return ccqGetLastNBroadcastsResponse

    def _create_serializable_msg(self):
        return ccqGetLastNBroadcastsResponse.msg_type()()


if __name__ == '__main__':

    try:
        module = ConsciousContentsQueue()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
