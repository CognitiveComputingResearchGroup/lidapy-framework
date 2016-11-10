from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "environment"

# Topics used by this module

# You may or may not want to use this module. This module will facilitate general
# pre-processing of messages between the agent and the actual environment.
# If you do not have a 3rd party or pre-built environment this module can be
# extended into another class to create your custom environment.

#TODO Think about adding 'env-sen' 'env-mot' as suffixes to topics for auto subscription

class Environment(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(Environment, self).__init__(name, **kwargs)

        # TODO get topic names from conf file and generate topics \
        # TODO for the user based on msgtype specified as \
        # TODO      <(sensory/motor)-param-name> = <topic-name>, <msg-type>

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        #TODO Add all sensory topics as published.

    def add_subscribers(self):
        #TODO Add all motor topics as subscribed

    def call(self):
        pass
