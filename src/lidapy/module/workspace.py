#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Coalitions
from lidapy.framework.msg import built_in_topics


class WorkspaceModule(FrameworkModule):
    def __init__(self):
        super(WorkspaceModule, self).__init__("WorkspaceModule")

    # Override this method to add more publishers
    def add_publishers(self):
        super(WorkspaceModule, self).add_publisher(built_in_topics["/lida/workspace_coalitions"])
        super(WorkspaceModule, self).add_publisher(built_in_topics["/lida/workspace_cues"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(WorkspaceModule, self).add_subscriber(built_in_topics["/lida/percepts"])
        super(WorkspaceModule, self).add_subscriber(built_in_topics["/lida/spatial_maps"])
        super(WorkspaceModule, self).add_subscriber(built_in_topics["/lida/episodes"])
        super(WorkspaceModule, self).add_subscriber(built_in_topics["/lida/global_broadcast"])

    def advance(self):
        self.logger.debug("Inside advance")

        next_percept = super(WorkspaceModule, self).get_next_msg("/lida/percepts")

        if next_percept is not None:
            coalitions = Coalitions()
            coalitions.id = next_percept.id

            self.publishers["/lida/workspace_coalitions"].publish(coalitions)


if __name__ == '__main__':

    try:
        module = WorkspaceModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
