from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import FrameworkTopic
from lidapy_rosdeps.srv import GenericService, GenericServiceResponse

# By default, the name of the module is the name of the ros node; however, this
# behavior can be overridden by passing a name to the initializer.
MODULE_NAME = "current_situational_model"

# Topics used by this module
WORKSPACE_CUES_TOPIC = FrameworkTopic("workspace_cues")


class CurrentSituationalModel(FrameworkModule):
    def __init__(self):
        super(CurrentSituationalModel, self).__init__()

        self.add_publishers([WORKSPACE_CUES_TOPIC])

        self.add_service("add_csm_content", GenericService, self.receive_add_csm_content_request)
        self.add_service("find_csm_content", GenericService, self.receive_find_csm_content_request)
        self.add_service("list_csm_content", GenericService, self.receive_list_csm_content_request)
        self.add_service("update_csm_content", GenericService, self.receive_update_csm_content_request)

    @classmethod
    def get_module_name(cls):
        return MODULE_NAME

    def receive_add_csm_content_request(self, request):
        # type: (csmAddContentRequest) -> csmAddContentResponse
        self.logger.debug("Receiving add_csm_content request: {}".format(request))

        # TODO: Adding content will initiatiate a publication to workspace cues

        return GenericServiceResponse()

    def receive_find_csm_content_request(self, request):
        # type: (csmFindContentRequest) -> csmFindContentResponse
        self.logger.debug("Receiving find_csm_content request: {}".format(request))

        return GenericServiceResponse()

    def receive_list_csm_content_request(self, request):
        # type: (csmListContentRequest) -> csmListContentResponse
        self.logger.debug("Receiving list_csm_content request: {}".format(request))

        return GenericServiceResponse()

    def receive_update_csm_content_request(self, request):
        # type: (csmUpdateContentRequest) -> csmUpdateContentResponse
        self.logger.debug("Receiving update_csm_content request: {}".format(request))

        # TODO: Updating content will initiatiate a publication to workspace cues

        return GenericServiceResponse()

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

    def __getitem__(self, item):
        pass
