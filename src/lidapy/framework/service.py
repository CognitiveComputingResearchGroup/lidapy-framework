from lidapy.framework.shared import FrameworkObject


class FrameworkService(FrameworkObject):
    def __init__(self, service_name, service_class, callback):
        super(FrameworkService, self).__init__()

        self.service_name = service_name
        self.service_class = service_class
        self.callback = callback

        # self.logger = FrameworkDependency("logger").resolve()
        # self.comm_proxy = FrameworkDependency("ipc_proxy").resolve()

        self.logger.info("Registering new service [{}]".format(self.service_name))

        self._service = self.ipc_proxy.get_service(self.service_name,
                                                   self.service_class,
                                                   self.callback)


class FrameworkServiceClient(FrameworkObject):
    def __init__(self, service_name, service_class):
        super(FrameworkServiceClient, self).__init__()

        self.service_name = service_name
        self.service_class = service_class

        # self.logger = FrameworkDependency("logger").resolve()
        # self.comm_proxy = FrameworkDependency("ipc_proxy").resolve()

    def get_service_proxy(self):
        return self.ipc_proxy.get_service_proxy(self.service_name, self.service_class)
