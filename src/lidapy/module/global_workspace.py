#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Coalitions
from lidapy.framework.msg import built_in_topics


class GlobalWorkspace(FrameworkModule):
    def __init__(self):
        super(GlobalWorkspace, self).__init__("GlobalWorkspace", decayable=True)

        # Override this method to add more publishers
    def add_publishers(self):
        super(GlobalWorkspace, self).add_publisher(built_in_topics["global_broadcast"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(GlobalWorkspace, self).add_subscriber(built_in_topics["workspace_coalitions"])

    def advance(self):
        super(GlobalWorkspace, self).advance()

        next_coalitions = super(GlobalWorkspace, self).get_next_msg("workspace_coalitions")

        if next_coalitions is not None:
            global_broadcast = Coalitions()
            global_broadcast.id = next_coalitions.id

            self.publishers["global_broadcast"].publish(global_broadcast)

if __name__ == '__main__':

    try:
        module = GlobalWorkspace()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
