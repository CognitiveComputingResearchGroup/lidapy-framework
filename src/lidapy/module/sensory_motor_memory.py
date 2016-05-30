#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class SensoryMotorMemory(FrameworkModule):
    def __init__(self, **kwargs):
        super(SensoryMotorMemory, self).__init__("SensoryMotorMemory", decayable=True, **kwargs)

    # Override this method to add more publishers
    def add_publishers(self):
        pass

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(SensoryMotorMemory, self).add_subscriber(built_in_topics["selected_behaviors"])
        super(SensoryMotorMemory, self).add_subscriber(built_in_topics["global_broadcast"])
        super(SensoryMotorMemory, self).add_subscriber(built_in_topics["dorsal_stream"])

    # Must be overridden
    def call(self):
        super(SensoryMotorMemory, self).call()


if __name__ == '__main__':

    try:
        module = SensoryMotorMemory()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
