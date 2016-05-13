#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Coalitions
from lidapy.framework.msg import built_in_topics


class GlobalWorkspaceModule(FrameworkModule):
    def __init__(self):
        super(GlobalWorkspaceModule, self).__init__("GlobalWorkspaceModule")

        # Override this method to add more publishers
    def add_publishers(self):
        super(GlobalWorkspaceModule, self).add_publisher(built_in_topics["/lida/global_broadcast"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(GlobalWorkspaceModule, self).add_subscriber(built_in_topics["/lida/workspace_coalitions"])

    def advance(self):
        next_coalitions = super(GlobalWorkspaceModule, self).get_next_msg("/lida/workspace_coalitions")

        if next_coalitions is not None:
            global_broadcast = Coalitions()
            global_broadcast.id = next_coalitions.id

            super(GlobalWorkspaceModule, self).publish("/lida/global_broadcast", global_broadcast)

if __name__ == '__main__':

    try:
        module = GlobalWorkspaceModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
