from collections import deque
from itertools import islice

from lida.srv import ccqGetLastNBroadcasts, ccqGetLastNBroadcastsResponse

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class ConsciousContentsQueue(FrameworkModule):
    def __init__(self, **kwargs):
        super(ConsciousContentsQueue, self).__init__("ConsciousContentsQueue", decayable=True, **kwargs)

        self.max_queue_size = self.config.get_param("ConsciousContentsQueue",
                                                    "max_queue_size", 10)

        self.queue = deque(maxlen=self.max_queue_size)

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(ConsciousContentsQueue, self).add_subscriber(built_in_topics["global_broadcast"])

    # Override this method to add more services
    def add_services(self):
        super(ConsciousContentsQueue, self).add_service("get_last_n_broadcasts",
                                                        ccqGetLastNBroadcasts,
                                                        self.process_last_n_broadcasts_request)

    def process_last_n_broadcasts_request(self, request):

        queue_size = len(self.queue)
        if request.last_n > queue_size:
            request.last_n = queue_size

        response = ccqGetLastNBroadcastsResponse()
        response.items = list(islice(self.queue, queue_size - request.last_n, queue_size))

        return response

    def call(self):
        super(ConsciousContentsQueue, self).call()

        broadcast = super(ConsciousContentsQueue, self).get_next_msg("global_broadcast")

        if broadcast is not None:
            self.queue.append(broadcast)
