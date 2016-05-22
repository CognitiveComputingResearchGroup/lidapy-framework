#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Behaviors
from lidapy.framework.msg import built_in_topics


class ProceduralMemoryModule(FrameworkModule):
    def __init__(self):
        super(ProceduralMemoryModule, self).__init__("ProceduralMemoryModule", decayable=True)

    # Override this method to add more publishers
    def add_publishers(self):
        super(ProceduralMemoryModule, self).add_publisher(built_in_topics["candidate_behaviors"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(ProceduralMemoryModule, self).add_subscriber(built_in_topics["global_broadcast"])

    def advance(self):
        super(ProceduralMemoryModule, self).advance()

        next_broadcast = super(ProceduralMemoryModule, self).get_next_msg("global_broadcast")

        if next_broadcast is not None:
            behaviors = Behaviors()
            behaviors.id = next_broadcast.id

            self.publishers["candidate_behaviors"].publish(behaviors)

if __name__ == '__main__':

    try:
        module = ProceduralMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
