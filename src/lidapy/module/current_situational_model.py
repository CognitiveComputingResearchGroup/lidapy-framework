from lidapy_rosdeps.srv import csmAddContent, csmAddContentRequest, csmAddContentResponse
from lidapy_rosdeps.srv import csmFindContent, csmFindContentRequest, csmFindContentResponse
from lidapy_rosdeps.srv import csmListContent, csmListContentRequest, csmListContentResponse
from lidapy_rosdeps.srv import csmUpdateContent, csmUpdateContentRequest, csmUpdateContentResponse

from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import built_in_topics
from lidapy.util import logger

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "current_situational_model"


class CurrentSituationalModel(FrameworkModule):
    def __init__(self, name=MODULE_NAME, **kwargs):
        super(CurrentSituationalModel, self).__init__(name, decayable=True, **kwargs)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def add_publishers(self):
        super(CurrentSituationalModel, self).add_publisher(built_in_topics["workspace_cues"])

    def add_services(self):
        super(CurrentSituationalModel, self).add_service("add_csm_content", csmAddContent,
                                                         self.receive_add_csm_content_request)
        super(CurrentSituationalModel, self).add_service("find_csm_content", csmFindContent,
                                                         self.receive_find_csm_content_request)
        super(CurrentSituationalModel, self).add_service("list_csm_content", csmListContent,
                                                         self.receive_list_csm_content_request)
        super(CurrentSituationalModel, self).add_service("update_csm_content", csmUpdateContent,
                                                         self.receive_update_csm_content_request)

    def get_next_msg(self, topic):
        return super(CurrentSituationalModel, self).get_next_msg(topic)

    def publish(self, topic, msg):
        super(CurrentSituationalModel, self).publish(topic, msg)

    def receive_add_csm_content_request(self, request):
        # type: (csmAddContentRequest) -> csmAddContentResponse
        logger.debug("Receiving add_csm_content request: {}".format(request))

        # TODO: Adding content will initiatiate a publication to workspace cues

        return csmAddContentResponse()

    def receive_find_csm_content_request(self, request):
        # type: (csmFindContentRequest) -> csmFindContentResponse
        logger.debug("Receiving find_csm_content request: {}".format(request))

        return csmFindContentResponse()

    def receive_list_csm_content_request(self, request):
        # type: (csmListContentRequest) -> csmListContentResponse
        logger.debug("Receiving list_csm_content request: {}".format(request))

        return csmListContentResponse()

    def receive_update_csm_content_request(self, request):
        # type: (csmUpdateContentRequest) -> csmUpdateContentResponse
        logger.debug("Receiving update_csm_content request: {}".format(request))

        # TODO: Updating content will initiatiate a publication to workspace cues

        return csmUpdateContentResponse()

    def call(self):
        pass


class WorkspaceBuffer(object):
    def __init__(self):
        pass

    def add(self, obj):
        pass

    def update(self, obj):
        pass

    def get(self, obj):
        pass

    def remove(self, obj):
        pass

    def decay(self, obj):
        pass

    def __init__(self):
        pass

    def __getitem__(self, item):
        pass
