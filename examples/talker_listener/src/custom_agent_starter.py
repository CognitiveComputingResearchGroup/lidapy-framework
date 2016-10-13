# To enable finding the lidapy package
import os, sys, inspect

# realpath() will make your script run, even if you symlink it :)
cmd_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile(inspect.currentframe()))[0]))
if "examples" in cmd_folder:
    cmd_folder = cmd_folder[:cmd_folder.index("examples")]
    cmd_folder = os.path.join(cmd_folder, "src")
print cmd_folder
if cmd_folder not in sys.path:
    sys.path.insert(0, cmd_folder)

# Start loading the lidapy packages
from lidapy.framework.agent_starter import AgentStarter
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic
from lidapy.util import logger

# Standard message supported by ROS (Robot Operating System)
from std_msgs.msg import String

# Module names.  These are used for the names of the ROS nodes, and for
# registering the modules with agent_starter.
TALKER_MODULE_NAME = "talker"
LISTENER_MODULE_NAME = "listener"

# Topics used in this example
TALKER_LISTENER_TOPIC = FrameworkTopic("lida/talker_listener_example", String, use_serializer=True)


class TalkerModule(FrameworkModule):
    def __init__(self, **kwargs):
        super(TalkerModule, self).__init__(TALKER_MODULE_NAME, **kwargs)

    @classmethod
    def get_module_name(cls):
        return TALKER_MODULE_NAME

    def add_publishers(self):
        super(TalkerModule, self).add_publisher(TALKER_LISTENER_TOPIC)

    # The call method is executed automatically every (1/rate_in_hz) seconds.  The
    # rate_in_hz parameter is defined in the agent.conf configuration file.
    def call(self):

        # Read message text from agent.conf
        msg = self.config.get_param(TALKER_MODULE_NAME, "message")

        # Write the message to the logger (/rosout)
        logger.info("Sending message: {}".format(msg))

        # Publish the message over the talker/listener topic
        super(TalkerModule, self).publish(TALKER_LISTENER_TOPIC, msg)


class ListenerModule(FrameworkModule):
    def __init__(self, **kwargs):
        super(ListenerModule, self).__init__(LISTENER_MODULE_NAME, **kwargs)

    @classmethod
    def get_module_name(cls):
        return LISTENER_MODULE_NAME

    def add_subscribers(self):
        super(ListenerModule, self).add_subscriber(TALKER_LISTENER_TOPIC)

    # The call method is executed automatically every (1/rate_in_hz) seconds.  The
    # rate_in_hz parameter is defined in the agent.conf configuration file.
    def call(self):

        # Pulls a recently received message from the message queue that is
        # maintained in FrameworkModule (parent class)
        msg = super(ListenerModule, self).get_next_msg(TALKER_LISTENER_TOPIC)

        if msg is not None:
            logger.info("Received message: {}".format(msg))


if __name__ == '__main__':

    try:
        # Execute custom_agent_starter.py with "-h" option for help on usage.
        starter = AgentStarter()

        # Registers TalkerModule and ListenerModule with the agent_starter.  After
        # registration, these modules can be invoked from a roslaunch file
        # (or command-line) using "-m <module name>" (see agent.launch for more details)
        starter.add_module(TalkerModule)
        starter.add_module(ListenerModule)

        # Launches a ROS node for the specified module.  Note that only one ROS node
        # will be launched per invocation of custom_agent_starter.py.
        starter.start()

    except Exception as e:
        print e

    finally:
        pass