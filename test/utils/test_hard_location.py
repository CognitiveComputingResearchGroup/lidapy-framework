import unittest
from utils import isdm
import numpy as np
import os
import shutil


class HardLocationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.mkdir(isdm.memory_location)

    def test_uncached_write(self):
        isdm.IntegerSDM.cached=False
        vector = isdm.MCRVector(np.array([5]*isdm.ndim))
        location = isdm.HardLocation('test_write')
        counters_before = location.counters
        location.write(vector)
        counters_after = location.counters

        for i, dim in enumerate(vector.dims):
            self.assertEqual(counters_after[i][dim]-counters_before[i][dim], 1)

    def test_cached_write(self):
        isdm.IntegerSDM.cached = True
        vector = isdm.MCRVector(np.array([5]*isdm.ndim))
        location = isdm.HardLocation('test_write')
        counters_before = location.counters
        location.write(vector)
        counters_after = location.counters

        for i, dim in enumerate(vector.dims):
            self.assertEqual(counters_after[i][dim]-counters_before[i][dim], 1)

    def test_first_call_to_counters(self):
        loc_name = 'test_first_call'
        location = isdm.HardLocation(loc_name)
        before_call = isdm.IntegerSDM.retrieve_counters(loc_name)
        self.assertIsNone(before_call)

    def test_second_call_to_counters(self):
        loc_name = 'test_second_call'
        location = isdm.HardLocation(loc_name)
        counters_before = location.counters
        counters_after = isdm.IntegerSDM.retrieve_counters(loc_name)
        self.assertIsNotNone(counters_after)
        for cb, ca in zip(counters_before, counters_after):
            for c in zip(cb, ca):
                self.assertEqual(c[0], c[1])

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(isdm.memory_location)

