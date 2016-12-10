import unittest

from lidapy.framework.shared import Activatable, CognitiveContent, CognitiveContentStructure, \
    CognitiveContentStructureIterator


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

        # self.fail("FAILURE:  Nah, just kidding.  Remove this line in test script.")
