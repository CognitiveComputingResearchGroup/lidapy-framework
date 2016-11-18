import unittest

from lidapy.framework.shared import FrameworkDependencyService
from lidapy.util.logger import ConsoleLogger
from lidapy.util.comm import LocalCommunicationProxy


class StructureBuildingCodeletTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fd = FrameworkDependencyService()

        fd["logger"] = ConsoleLogger()
        fd["ipc_proxy"] = LocalCommunicationProxy()

    @classmethod
    def tearDownClass(cls):
        pass

    def test(self):
        pass
