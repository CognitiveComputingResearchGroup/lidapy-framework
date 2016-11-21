import unittest

from lidapy.framework.shared import Activatable, CognitiveContent, CognitiveContentStructure, \
    CognitiveContentStructureIterator, FrameworkDependencyService, FrameworkDependency
from lidapy.util.comm import LocalCommunicationProxy
from lidapy.util.logger import ConsoleLogger
from lidapy.util.meta import Singleton


class FrameworkDependencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Remove any singleton instances of FrameworkDependencyService
        # to guarantee that test cases are independent
        fd = FrameworkDependencyService()

        fd["logger"] = ConsoleLogger()
        fd["ipc_proxy"] = LocalCommunicationProxy()
        fd["string_dep"] = "just a string"

    @classmethod
    def tearDownClass(cls):
        pass

    def test_satisfied(self):
        logger_depend = FrameworkDependency("logger")
        self.assertTrue(logger_depend.is_satisfied())

        non_existent_dependency = FrameworkDependency("nobody home")
        self.assertTrue(not non_existent_dependency.is_satisfied())

    def test_resolve(self):
        logger_depend = FrameworkDependency("logger")
        logger = logger_depend.resolve()

        self.assertTrue(type(logger) is ConsoleLogger)

        non_existent_dependency = FrameworkDependency("nobody home")
        try:
            non_existent = non_existent_dependency.resolve()
            self.fail("Failed to generate exception from non-existent dependency")
        except:
            pass


class FrameworkDependencyServiceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    @classmethod
    def tearDown(cls):
        # Removes all previously created singleton objects between tests
        Singleton._clear()

    def test_singleton_property(self):

        # Verify that singleton property is working (separately created dependency
        # services must be identical)
        fds1 = FrameworkDependencyService()
        fds2 = FrameworkDependencyService()

        self.assertTrue(fds1 is fds2)

    def test_add_dependency(self):
        fds = FrameworkDependencyService()

        # Verify that new dependency was added and length updated
        fds["dependency1"] = "dependency1"
        self.assertEqual(len(fds), 1)

        # Verify that 2nd dependency was added, length updated,
        fds["dependency2"] = "dependency2"
        self.assertEqual(len(fds), 2)

        # Verify that disallowing "overrides" by default and raising
        # an exception if multiple dependency sets occur
        try:
            fds["dependency1"] = "dependency1"
            self.fail("Failed to generate exception when not allowing overrides and duplicate set")
        except:
            pass

        fds.allow_overrides = True

        # Verify that multiple sets allowed when allowing overrides
        try:
            fds["dependency1"] = "dependency1"
        except:
            self.fail("Should not generate exception when duplicate sets and allowing overrides")

    def test_get_dependency(self):
        fds = FrameworkDependencyService()

        # Verify content from get matches expected value
        fds["dependency1"] = "dependency1_value"
        self.assertEqual(fds["dependency1"], "dependency1_value")

        # Verify non-existent dependency returns exception
        try:
            d = fds["nobody home"]
            self.fail("Failed to generate expected exception when retrieving non-existent dependency")
        except:
            pass

    def test_has_dependency(self):
        fds = FrameworkDependencyService()

        # Verify has reports True when dependency was added
        fds["dependency1"] = "dependency1"
        self.assertTrue(fds.has("dependency1"))

        # Verify has reports False when dependency not added
        self.assertTrue(not fds.has("nobody home"))


class ActivatableTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        # Verify initial activation is 0.0
        a = Activatable()
        self.assertEqual(a.activation, 0.0)

        # Verify initial incentive salience is 0.0
        a = Activatable()
        self.assertEqual(a.incentive_salience, 0.0)

        # Verify initial base_level_activation is 0.0
        a = Activatable()
        self.assertEqual(a.base_level_activation, 0.0)

        # Verify that initial activation can be set during initialization
        a = Activatable(0.25, 0.5, 0.05)
        self.assertEqual(a.activation, 0.25)

        # Verify that initial incentive salience can be set during initialization
        a = Activatable(0.25, 0.5, 0.05)
        self.assertEqual(a.incentive_salience, 0.5)

        # Verify that initial base_level_activation can be set during initialization
        a = Activatable(0.25, 0.5, 0.05)
        self.assertEqual(a.base_level_activation, 0.05)

        # Verify small change of activation and incentive salience within
        # expected range bounds works
        a = Activatable()
        a.activation += 0.25
        self.assertEqual(a.activation, 0.25)

        a.activation -= 0.1
        self.assertEqual(a.activation, 0.15)

        a = Activatable()
        a.incentive_salience += 0.25
        self.assertEqual(a.incentive_salience, 0.25)

        a.incentive_salience -= 0.1
        self.assertEqual(a.incentive_salience, 0.15)

        a = Activatable()
        a.base_level_activation += 0.25
        self.assertEqual(a.base_level_activation, 0.25)

        a.base_level_activation -= 0.1
        self.assertEqual(a.base_level_activation, 0.15)

        # Verify that reduction of activation and incentive salience and base_level_activation
        # to below lower bound is scaled to lower bound
        a = Activatable()
        a.activation -= 0.25
        self.assertEqual(a.activation, 0.0)

        a = Activatable()
        a.incentive_salience -= 0.25
        self.assertEqual(a.incentive_salience, 0.0)

        a = Activatable()
        a.base_level_activation -= 0.25
        self.assertEqual(a.base_level_activation, 0.0)

        # Verify that increase of activation and incentive salience and base_level_activation
        # to above upper bound is scaled to upper bound
        a = Activatable()
        a.activation += 2.0
        self.assertEqual(a.activation, 1.0)

        a = Activatable()
        a.incentive_salience += 2.0
        self.assertEqual(a.incentive_salience, 1.0)

        a = Activatable()
        a.base_level_activation += 2.0
        self.assertEqual(a.base_level_activation, 1.0)


class CognitiveContentStructureIteratorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        it = CognitiveContentStructureIterator([CognitiveContent(value) for value in range(1, 5)])

        expected_value = 1
        for actual in it:
            self.assertEqual(expected_value, actual.value)
            expected_value += 1


class CognitiveContentStructureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):

        # Check that empty CognitiveContentStructure has len() = 0
        ccs = CognitiveContentStructure()
        self.assertEqual(len(ccs), 0)

        # Check that passing an iterable with N elements results in
        # len(CognitiveContentStructure) = N
        ccs = CognitiveContentStructure([CognitiveContent(v) for v in range(10)])
        self.assertEqual(len(ccs), 10)

        # Return TypeError if a non-iterable is passed
        with self.assertRaises(TypeError):
            CognitiveContentStructure(CognitiveContent("nope"))

        # Return TypeError if iterable contains non-CognitiveContent
        with self.assertRaises(TypeError):
            CognitiveContentStructure(range(10))

    def test_insert(self):

        # Check that adding an element increases the len() by 1
        ccs = CognitiveContentStructure()

        ccs.insert(CognitiveContent(1))
        self.assertEqual(len(ccs), 1)

        ccs.insert(CognitiveContent(2))
        self.assertEqual(len(ccs), 2)

    def test_add(self):
        ccs1 = CognitiveContentStructure([CognitiveContent(v) for v in range(0, 5)])
        ccs2 = CognitiveContentStructure([CognitiveContent(v) for v in range(5, 10)])

        ccs_result = ccs1 + ccs2

        value_list = []
        for cc in ccs_result:
            value_list.append(cc.value)

        self.assertEqual(len(value_list), 10)
        self.assertEqual(set(value_list), set(range(10)))

    def test_iadd(self):
        ccs1 = CognitiveContentStructure([CognitiveContent(v) for v in range(0, 5)])
        ccs2 = CognitiveContentStructure([CognitiveContent(v) for v in range(5, 10)])

        ccs1 += ccs2

        value_list = []
        for cc in ccs1:
            value_list.append(cc.value)

        self.assertEqual(len(value_list), 10)
        self.assertEqual(set(value_list), set(range(10)))

        ccs1 = CognitiveContentStructure([CognitiveContent(v) for v in range(0, 5)])
        ccs1 += [CognitiveContent(v) for v in range(5, 10)]

        value_list = []
        for cc in ccs1:
            value_list.append(cc.value)

        self.assertEqual(len(value_list), 10)
        self.assertEqual(set(value_list), set(range(10)))

    def test_radd(self):
        ccs2 = CognitiveContentStructure([CognitiveContent(v) for v in range(5, 10)])

        ccs_result = [CognitiveContent(v) for v in range(0, 5)] + ccs2

        value_list = []
        for cc in ccs_result:
            value_list.append(cc.value)

        self.assertEqual(len(value_list), 10)
        self.assertEqual(set(value_list), set(range(10)))

    def test_remove(self):

        # Check that removing an element decreases the len() by 1
        ccs = CognitiveContentStructure()
        self.assertEqual(len(ccs), 0)
        cc = CognitiveContent(1)
        ccs.insert(cc)
        self.assertEqual(len(ccs), 1)
        ccs.remove(cc)
        self.assertEqual(len(ccs), 0)

        # Check that removing an non-existent element is ignored
        ccs.remove(cc)
        self.assertEqual(len(ccs), 0)

    def test_iter(self):
        ccs = CognitiveContentStructure()

        cc_list = [CognitiveContent(1),
                   CognitiveContent(2),
                   CognitiveContent(3)]

        for cc in cc_list:
            ccs.insert(cc)

        self.assertEqual(len(cc_list), len(ccs))

        # Check expected number of loop iterations
        n = 0
        for cc in ccs:
            n += 1

        self.assertEqual(n, len(cc_list))

        # Check content matches expectation
        for cc in ccs:
            if cc not in cc_list:
                self.fail("CognitiveContentStructure iterator returned unexpected content")

            # Remove found CognitiveContent from original list.  This will
            # be used to guarantee that the iterator returned exactly the
            # expected content without duplicates or omissions
            cc_list.remove(cc)

        # If all CognitiveContent objects returned then the original
        # list should have no elements.
        self.assertEqual(len(cc_list), 0)

        ccs = CognitiveContentStructure()
        for cc in ccs:
            self.fail("Should not have entered loop.  CognitiveContentStructure is empty")

    def test_apply(self):
        ccs = CognitiveContentStructure()

        cc1 = CognitiveContent(1)
        cc2 = CognitiveContent(2)
        cc3 = CognitiveContent(3)

        ccs.insert(cc1)
        ccs.insert(cc2)
        ccs.insert(cc3)

        self.assertEqual(len(ccs), 3)

        def multiply_value_by_2(cc):
            cc.value *= 2

        ccs.apply(multiply_value_by_2)

        self.assertEqual(cc1.value, 2)
        self.assertEqual(cc2.value, 4)
        self.assertEqual(cc3.value, 6)

    def test_remove_all_matches(self):
        ccs = CognitiveContentStructure()

        for i in range(0, 100):
            ccs.insert(CognitiveContent(i))

        self.assertEqual(len(ccs), 100)

        ccs.remove_all_matches(lambda x: x.value >= 50)

        self.assertEqual(len(ccs), 50)
