#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Percepts
from lidapy.framework.msg import built_in_topics


class PerceptualAssociativeMemoryModule(FrameworkModule):
    def __init__(self):
        super(PerceptualAssociativeMemoryModule, self).__init__("PerceptualAssociativeMemoryModule")

    # Override this method to add more publishers
    def add_publishers(self):
        super(PerceptualAssociativeMemoryModule, self).add_publisher(built_in_topics["/lida/percepts"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(PerceptualAssociativeMemoryModule, self).add_subscriber(built_in_topics["/lida/detected_features"])
        super(PerceptualAssociativeMemoryModule, self).add_subscriber(built_in_topics["/lida/workspace_cues"])
        super(PerceptualAssociativeMemoryModule, self).add_subscriber(built_in_topics["/lida/global_broadcast"])

    def advance(self):
        next_features = super(PerceptualAssociativeMemoryModule, self).get_next_msg("/lida/detected_features")

        if next_features is not None:
            percepts = Percepts()
            percepts.id = next_features.id

            self.publishers["/lida/percepts"].publish(percepts)


if __name__ == '__main__':

    try:
        module = PerceptualAssociativeMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
