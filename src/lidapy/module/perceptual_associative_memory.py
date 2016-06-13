#!/usr/bin/env python

from lidapy.framework.agent_starter import AgentStarter
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class PerceptualAssociativeMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(PerceptualAssociativeMemory, self).__init__("PerceptualAssociativeMemory", decayable=True,
                                                          cueable=True, **kwargs)

    # Override this method to add more publishers
    def add_publishers(self):
        super(PerceptualAssociativeMemory, self).add_publisher(built_in_topics["percepts"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(PerceptualAssociativeMemory, self).add_subscriber(built_in_topics["detected_features"])
        super(PerceptualAssociativeMemory, self).add_subscriber(built_in_topics["workspace_cues"])
        super(PerceptualAssociativeMemory, self).add_subscriber(built_in_topics["global_broadcast"])

    def call(self):
        super(PerceptualAssociativeMemory, self).call()

        detected_features = super(PerceptualAssociativeMemory, self).get_next_msg("detected_features")

        if detected_features is not None:
            active_percepts = detected_features

            self.publishers["percepts"].publish(active_percepts)


if __name__ == '__main__':

    try:

        starter = AgentStarter()
        starter.start(module_name="PerceptualAssociativeMemory")

    except Exception as e:
        print e

    finally:
        pass
