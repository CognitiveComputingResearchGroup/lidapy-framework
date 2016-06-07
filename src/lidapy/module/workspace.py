#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics


class Workspace(FrameworkModule):
    def __init__(self, **kwargs):
        super(Workspace, self).__init__("Workspace", decayable=True, **kwargs)

    # Override this method to add more publishers
    def add_publishers(self):
        super(Workspace, self).add_publisher(built_in_topics["workspace_coalitions"])
        super(Workspace, self).add_publisher(built_in_topics["workspace_cues"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(Workspace, self).add_subscriber(built_in_topics["percepts"])
        super(Workspace, self).add_subscriber(built_in_topics["spatial_maps"])
        super(Workspace, self).add_subscriber(built_in_topics["episodes"])
        super(Workspace, self).add_subscriber(built_in_topics["global_broadcast"])

    def call(self):
        super(Workspace, self).call()

        percepts = super(Workspace, self).get_next_msg("percepts")

        if percepts is not None:
            workspace_coalitions = percepts

            self.publishers["workspace_coalitions"].publish(workspace_coalitions)


if __name__ == '__main__':

    try:
        module = Workspace()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
