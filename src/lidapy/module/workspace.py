from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.service import FrameworkServiceClient
from lidapy_rosdeps.srv import GenericService, GenericServiceRequest

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "workspace"

# Topics used by this module
WORKSPACE_COALITIONS_TOPIC = FrameworkTopic("workspace_coalitions")
WORKSPACE_CUES_TOPIC = FrameworkTopic("workspace_cues")
PERCEPTS_TOPIC = FrameworkTopic("percepts")
SPATIAL_MAPS_TOPIC = FrameworkTopic("spatial_maps")
EPISODES_TOPIC = FrameworkTopic("episodes")
GLOBAL_BROADCAST_TOPIC = FrameworkTopic("global_broadcast")


class Workspace(FrameworkModule):
    def __init__(self, **kwargs):
        super(Workspace, self).__init__(**kwargs)

        self.csm_add_content_srv_client \
            = FrameworkServiceClient("add_csm_content", GenericService).get_service_proxy()

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        self.add_publisher(WORKSPACE_COALITIONS_TOPIC)
        self.add_publisher(WORKSPACE_CUES_TOPIC)

    def add_subscribers(self):
        self.add_subscriber(PERCEPTS_TOPIC)
        self.add_subscriber(SPATIAL_MAPS_TOPIC)
        self.add_subscriber(EPISODES_TOPIC)
        self.add_subscriber(GLOBAL_BROADCAST_TOPIC)

    def call(self):
        percepts = self.get_next_msg(PERCEPTS_TOPIC)

        if percepts is not None:
            request = GenericServiceRequest()

            # TODO: Populate request details.
            self.csm_add_content_srv_client(request)
