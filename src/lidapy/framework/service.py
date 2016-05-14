from lidapy.util import logger, comm


class FrameworkService(object):
    def __init__(self, service_name, service_class, callback):
        super(FrameworkService, self).__init__()

        self.service_name = service_name
        self.service_class = service_class
        self.callback = callback

        logger.info("Registering new service [{}]".format(self.service_name))

        self._service = comm.get_service(self.service_name,
                                         self.service_class,
                                         self.callback)


class FrameworkServiceClient(object):
    def __init__(self, service_name, service_class):
        super(FrameworkServiceClient, self).__init__()

        self.service_name = service_name
        self.service_class = service_class

    def get_service_proxy(self):
        return comm.get_service_proxy(self.service_name, self.service_class)
