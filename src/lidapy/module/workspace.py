from lidapy_rosdeps.srv import GenericService, GenericServiceRequest

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics
from lidapy.framework.service import FrameworkServiceClient

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "workspace"

# Topics used by this module
WORKSPACE_COALITIONS_TOPIC = built_in_topics["workspace_coalitions"]
WORKSPACE_CUES_TOPIC = built_in_topics["workspace_cues"]
PERCEPTS_TOPIC = built_in_topics["percepts"]
SPATIAL_MAPS_TOPIC = built_in_topics["spatial_maps"]
EPISODES_TOPIC = built_in_topics["episodes"]
GLOBAL_BROADCAST_TOPIC = built_in_topics["global_broadcast"]


class Workspace(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(Workspace, self).__init__(name, **kwargs)

        self.csm_add_content_srv_client = FrameworkServiceClient("add_csm_content", GenericService).get_service_proxy()

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        super(Workspace, self).add_publisher(WORKSPACE_COALITIONS_TOPIC)
        super(Workspace, self).add_publisher(WORKSPACE_CUES_TOPIC)

    def add_subscribers(self):
        super(Workspace, self).add_subscriber(PERCEPTS_TOPIC)
        super(Workspace, self).add_subscriber(SPATIAL_MAPS_TOPIC)
        super(Workspace, self).add_subscriber(EPISODES_TOPIC)
        super(Workspace, self).add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def get_next_msg(self, topic):
        return super(Workspace, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(Workspace, self).publish(topic, msg)

    def call(self):
        percepts = self.get_next_msg(PERCEPTS_TOPIC)

        if percepts is not None:
            request = GenericServiceRequest()

            # TODO: Populate request details.
            self.csm_add_content_srv_client(request)
