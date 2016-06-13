#!/usr/bin/env python

from lidapy.framework.agent_starter import AgentStarter
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class SensoryMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(SensoryMemory, self).__init__("SensoryMemory", decayable=True, **kwargs)

        self.add_publishers()
        self.add_subscribers()

    # Override this method to add more publishers
    def add_publishers(self):
        super(SensoryMemory, self).add_publisher(built_in_topics["dorsal_stream"])
        super(SensoryMemory, self).add_publisher(built_in_topics["ventral_stream"])
        super(SensoryMemory, self).add_publisher(built_in_topics["detected_features"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(SensoryMemory, self).add_subscriber(built_in_topics["global_broadcast"])

    # Must be overridden
    def call(self):
        super(SensoryMemory, self).call()


if __name__ == '__main__':

    try:

        starter = AgentStarter()
        starter.start(module_name="SensoryMemory")

    except Exception as e:
        print e

    finally:
        pass
