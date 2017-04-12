import unittest
import numpy as np
from utils import isdm
import os
import shutil
from utils.isdm import HardLocation, IntegerSDM


class IntegerSDMTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.memory = IntegerSDM(100000)

    def test_storage(self):

        counters = HardLocation.get_empty_counter()
        counters[0][0]=5

        location_name = 'test_storage'
        IntegerSDM.store_counters(location_name, counters)

        retreived = IntegerSDM.retrieve_counters(location_name)

        self.assertEqual(retreived[0][0], 5)

    def test_write(self):
        pass


    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(isdm.memory_location)

