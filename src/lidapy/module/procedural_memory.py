#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Behaviors
from lidapy.framework.msg import built_in_topics


class ProceduralMemoryModule(FrameworkModule):
    def __init__(self):
        super(ProceduralMemoryModule, self).__init__("ProceduralMemoryModule")

    # Override this method to add more publishers
    def add_publishers(self):
        super(ProceduralMemoryModule, self).add_publisher(built_in_topics["/lida/candidate_behaviors"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(ProceduralMemoryModule, self).add_subscriber(built_in_topics["/lida/global_broadcast"])

    def advance(self):
        self.logger.debug("Inside advance")

        next_broadcast = super(ProceduralMemoryModule, self).get_next_msg("/lida/global_broadcast")

        if next_broadcast is not None:
            behaviors = Behaviors()
            behaviors.id = next_broadcast.id

            self.publishers["/lida/candidate_behaviors"].publish(behaviors)

if __name__ == '__main__':

    try:
        module = ProceduralMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
