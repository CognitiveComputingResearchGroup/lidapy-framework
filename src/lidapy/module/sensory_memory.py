#!/usr/bin/env python

from random import randint

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Features
from lidapy.framework.msg import built_in_topics


class SensoryMemoryModule(FrameworkModule):
    def __init__(self):
        super(SensoryMemoryModule, self).__init__("SensoryMemoryModule")

        self.add_publishers()
        self.add_subscribers()

    # Override this method to add more publishers
    def add_publishers(self):
        super(SensoryMemoryModule, self).add_publisher(built_in_topics["/lida/dorsal_stream"])
        super(SensoryMemoryModule, self).add_publisher(built_in_topics["/lida/ventral_stream"])
        super(SensoryMemoryModule, self).add_publisher(built_in_topics["/lida/detected_features"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        pass

    def advance(self):
        msg = Features()
        msg.id = str(randint(0, 1e15 - 1))
        super(SensoryMemoryModule, self).publish("/lida/detected_features", msg)


if __name__ == '__main__':

    try:
        module = SensoryMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
