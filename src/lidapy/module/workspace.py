from lida.srv import csmAddContent, csmAddContentRequest

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics
from lidapy.framework.service import FrameworkServiceClient


class Workspace(FrameworkModule):
    def __init__(self, **kwargs):
        super(Workspace, self).__init__("Workspace", decayable=True, **kwargs)

        self.csm_add_content_srv_client = FrameworkServiceClient("add_csm_content", csmAddContent).get_service_proxy()

    # Override this method to add more publishers
    def add_publishers(self):
        super(Workspace, self).add_publisher(built_in_topics["workspace_coalitions"])

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
            request = csmAddContentRequest()
            request.content = [percepts]

            self.csm_add_content_srv_client(request)
