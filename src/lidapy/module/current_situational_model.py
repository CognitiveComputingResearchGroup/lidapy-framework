#!/usr/bin/env python

from lida.srv import csmAddContent, csmAddContentRequest, csmAddContentResponse
from lida.srv import csmFindContent, csmFindContentRequest, csmFindContentResponse
from lida.srv import csmListContent, csmListContentRequest, csmListContentResponse
from lida.srv import csmUpdateContent, csmUpdateContentRequest, csmUpdateContentResponse

from lidapy.framework.module import FrameworkModule


class CurrentSituationalModel(FrameworkModule):
    def __init__(self, **kwargs):
        super(CurrentSituationalModel, self).__init__("CurrentSituationalModel", decayable=True, **kwargs)

    def add_services(self):
        super(CurrentSituationalModel, self).add_service("add_csm_content", csmAddContent,
                                                         self.receive_add_csm_content_request)
        super(CurrentSituationalModel, self).add_service("find_csm_content", csmFindContent,
                                                         self.receive_find_csm_content_request)
        super(CurrentSituationalModel, self).add_service("list_csm_content", csmListContent,
                                                         self.receive_list_csm_content_request)
        super(CurrentSituationalModel, self).add_service("update_csm_content", csmUpdateContent,
                                                         self.receive_update_csm_content_request)

    def receive_add_csm_content_request(self, request):
        # type: (csmAddContentRequest) -> csmAddContentResponse
        self.logger.debug("Receiving add_csm_content request: {}".format(request))

    def receive_find_csm_content_request(self, request):
        # type: (csmFindContentRequest) -> csmFindContentResponse
        self.logger.debug("Receiving find_csm_content request: {}".format(request))

    def receive_list_csm_content_request(self, request):
        # type: (csmListContentRequest) -> csmListContentResponse
        self.logger.debug("Receiving list_csm_content request: {}".format(request))

    def receive_update_csm_content_request(self, request):
        # type: (csmUpdateContentRequest) -> csmUpdateContentResponse
        self.logger.debug("Receiving update_csm_content request: {}".format(request))

    def call(self):
        pass


if __name__ == '__main__':

    try:
        module = CurrentSituationalModel()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
