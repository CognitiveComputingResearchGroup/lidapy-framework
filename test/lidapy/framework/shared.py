import unittest

from lidapy.framework.shared import Activatable, CognitiveContent, CognitiveContentStructure, \
    CognitiveContentStructureIterator, FrameworkDependencyService, FrameworkDependency
from lidapy.util.comm import StubCommunicationProxy
from lidapy.util.logger import ConsoleLogger
from lidapy.util.meta import Singleton


class FrameworkDependencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fd = FrameworkDependencyService()

        fd["logger"] = ConsoleLogger()
        fd["ipc_proxy"] = StubCommunicationProxy()
        fd["string_dep"] = "just a string"

    @classmethod
    def tearDownClass(cls):
        pass

    def test_satisfied(self):
        logger_depend = FrameworkDependency("logger")
        assert (logger_depend.is_satisfied())

        non_existent_dependency = FrameworkDependency("nobody home")
        assert (not non_existent_dependency.is_satisfied())

    def test_resolve(self):
        logger_depend = FrameworkDependency("logger")
        logger = logger_depend.resolve()

        assert (type(logger) is ConsoleLogger)

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

        assert (fds1 is fds2)

    def test_add_dependency(self):
        fds = FrameworkDependencyService()

        # Verify that new dependency was added and length updated
        fds["dependency1"] = "dependency1"
        assert (len(fds) == 1)

        # Verify that 2nd dependency was added, length updated,
        fds["dependency2"] = "dependency2"
        assert (len(fds) == 2)

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
        assert (fds["dependency1"] == "dependency1_value")

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
        assert (fds.has("dependency1"))

        # Verify has reports False when dependency not added
        assert (not fds.has("nobody home"))


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
        assert (a.activation == 0.0)

        # Verify initial incentive salience is 0.0
        a = Activatable()
        assert (a.incentive_salience == 0.0)

        # Verify initial base_level_activation is 0.0
        a = Activatable()
        assert (a.base_level_activation == 0.0)

        # Verify that initial activation can be set during initialization
        a = Activatable(0.25, 0.5, 0.05)
        assert (a.activation == 0.25)

        # Verify that initial incentive salience can be set during initialization
        a = Activatable(0.25, 0.5, 0.05)
        assert (a.incentive_salience == 0.5)

        # Verify that initial base_level_activation can be set during initialization
        a = Activatable(0.25, 0.5, 0.05)
        assert (a.base_level_activation == 0.05)

        # Verify small change of activation and incentive salience within
        # expected range bounds works
        a = Activatable()
        a.activation += 0.25
        assert (a.activation == 0.25)

        a.activation -= 0.1
        assert (a.activation == 0.15)

        a = Activatable()
        a.incentive_salience += 0.25
        assert (a.incentive_salience == 0.25)

        a.incentive_salience -= 0.1
        assert (a.incentive_salience == 0.15)

        a = Activatable()
        a.base_level_activation += 0.25
        assert (a.base_level_activation == 0.25)

        a.base_level_activation -= 0.1
        assert (a.base_level_activation == 0.15)

        # Verify that reduction of activation and incentive salience and base_level_activation
        # to below lower bound is scaled to lower bound
        a = Activatable()
        a.activation -= 0.25
        assert (a.activation == 0.0)

        a = Activatable()
        a.incentive_salience -= 0.25
        assert (a.incentive_salience == 0.0)

        a = Activatable()
        a.base_level_activation -= 0.25
        assert (a.base_level_activation == 0.0)

        # Verify that increase of activation and incentive salience and base_level_activation
        # to above upper bound is scaled to upper bound
        a = Activatable()
        a.activation += 2.0
        assert (a.activation == 1.0)

        a = Activatable()
        a.incentive_salience += 2.0
        assert (a.incentive_salience == 1.0)

        a = Activatable()
        a.base_level_activation += 2.0
        assert (a.base_level_activation == 1.0)


class CognitiveContentStructureIteratorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        it = CognitiveContentStructureIterator(
            [CognitiveContent(1),
             CognitiveContent(2),
             CognitiveContent(3),
             CognitiveContent(4),
             CognitiveContent(5)])

        expected_value = 1
        for actual in it:
            assert (expected_value == actual.value)
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
        c = CognitiveContentStructure()
        assert (len(c)) == 0

        # Check that adding an element increases the len() by 1
        c.insert(CognitiveContent(1))
        assert (len(c) == 1)

        # Check that removing an element decreases the len() by 1
        c.remove(CognitiveContent(1))
        assert (len(c) == 0)
