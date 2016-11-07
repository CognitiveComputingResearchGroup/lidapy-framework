import unittest

from lidapy.framework.process import FrameworkThread
from lidapy.util.comm import StubCommunicationProxy
from lidapy.util.logger import ConsoleLogger


class FrameworkTaskTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):

        class callback():
            def __init__(self):
                self.count = 0

            def __call__(self, *args, **kwargs):
                self.count += 1

        callback_cls = callback()

        try:

            task = FrameworkThread(name="test task",
                                   callback=callback_cls,
                                   exec_count=100,
                                   rate_in_hz=1000,
                                   logger=ConsoleLogger(),
                                   comm_proxy=StubCommunicationProxy())

            # Start task in thread
            task.start()

            # Wait for thread to complete
            task.join()

            # Verify that execution count matches expected number of executions
            assert (100 == callback_cls.count)

        except Exception as e:
            self.fail("Unexpected Exception: {}".format(e))
