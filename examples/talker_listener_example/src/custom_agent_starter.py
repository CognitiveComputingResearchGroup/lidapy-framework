#! /usr/bin/env python

from lidapy.framework.agent_starter import AgentStarter
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic

from std_msgs.msg import String


TOPIC = FrameworkTopic("lida/comm", String)


class TalkerModule(FrameworkModule):
    def __init__(self):
        super(TalkerModule, self).__init__("Talker")
        super(TalkerModule, self).add_publisher(TOPIC)

    def call(self):
        next_msg = self.create_next_msg()
        self.logger.info("Sending message: {}".format(next_msg))
        self.publishers[TOPIC.topic_name].publish(next_msg)

    def create_next_msg(self):
        return String("Hello LidaPy!")


class ListenerModule(FrameworkModule):
    def __init__(self):
        super(ListenerModule, self).__init__("Listener")
        super(ListenerModule, self).add_subscriber(TOPIC)

    def call(self):
        msg = self.get_next_msg(TOPIC.topic_name)

        if msg is not None:
            self.logger.info("Receiving message: {}".format(msg))


if __name__ == '__main__':

    try:
        starter = AgentStarter()

        starter.add_module("talker", TalkerModule)
        starter.add_module("listener", ListenerModule)

        starter.start()

    except Exception as e:
        print e

    finally:
        pass