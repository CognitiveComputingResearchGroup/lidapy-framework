#!/usr/bin/env python

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Behaviors
from lidapy.framework.msg import built_in_topics


class ActionSelectionModule(FrameworkModule):
    def __init__(self):
        super(ActionSelectionModule, self).__init__("ActionSelectionModule")

    # Override this method to add more publishers
    def add_publishers(self):
        super(ActionSelectionModule, self).add_publisher(built_in_topics["/lida/selected_behaviors"])

    # Override this method to add more subscribers
    def add_subscribers(self):
        super(ActionSelectionModule, self).add_subscriber(built_in_topics["/lida/candidate_behaviors"])
        super(ActionSelectionModule, self).add_subscriber(built_in_topics["/lida/global_broadcast"])

    def advance(self):
        next_behavior = super(ActionSelectionModule, self).get_next_msg("/lida/candidate_behaviors")

        if next_behavior is not None:
            behaviors = Behaviors()
            behaviors.id = next_behavior.id

            self.publishers["/lida/selected_behaviors"].publish(behaviors)


if __name__ == '__main__':

    try:
        module = ActionSelectionModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
