#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class SensoryMotorMemoryModule(FrameworkModule):
    def __init__(self):
        super(SensoryMotorMemoryModule, self).__init__("SensoryMotorMemoryModule")

    # Override this method to add more publishers
    def add_publishers(self):
        pass

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(SensoryMotorMemoryModule, self).add_subscriber(built_in_topics["/lida/selected_behaviors"])
        super(SensoryMotorMemoryModule, self).add_subscriber(built_in_topics["/lida/global_broadcast"])
        super(SensoryMotorMemoryModule, self).add_subscriber(built_in_topics["/lida/dorsal_stream"])


if __name__ == '__main__':

    try:
        module = SensoryMotorMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
