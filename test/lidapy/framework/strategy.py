from nose.tools import raises
import unittest

from lidapy.framework.shared import CognitiveContent
from lidapy.framework.strategy import LinearDecayStrategy, LinearExciteStrategy, SigmoidDecayStrategy, \
    SigmoidExciteStrategy, MIN_ACTIVATION, MAX_ACTIVATION


class LinearExciteStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        strategy = LinearExciteStrategy(slope=0.1)
        self.assertEqual(strategy.slope, 0.1)

        with self.assertRaises(ValueError):
            LinearExciteStrategy(slope=0.0)

        with self.assertRaises(ValueError):
            LinearExciteStrategy(slope=-1.0)

    def test_get_next_value(self):
        strategy = LinearExciteStrategy(slope=0.1)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation=0.0, rate_in_hz=-0.1)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation=0.0, rate_in_hz=0.0)

        current_activation = 0.0
        for rate in [1.0, 10.0, 100.0, 1000.0]:
            next_value = strategy.get_next_value(current_activation=current_activation, rate_in_hz=rate)
            self.assertAlmostEqual(current_activation + (strategy.slope / rate), next_value)

        for current_activation in [0.0, 0.1, 0.5, 0.75, 0.9]:
            next_value = strategy.get_next_value(current_activation=current_activation, rate_in_hz=1.0)
            self.assertAlmostEqual(current_activation + strategy.slope, next_value)

        # Verify that MAX_ACTIVATION is enforced
        next_value = strategy.get_next_value(current_activation=MAX_ACTIVATION, rate_in_hz=1.0)
        self.assertAlmostEqual(MAX_ACTIVATION, next_value)


class LinearDecayStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        strategy = LinearDecayStrategy(slope=0.1)
        self.assertEqual(strategy.slope, 0.1)

        with self.assertRaises(ValueError):
            LinearDecayStrategy(slope=0.0)

        with self.assertRaises(ValueError):
            LinearDecayStrategy(slope=-1.0)

    def test_get_next_value(self):
        strategy = LinearDecayStrategy(slope=0.1)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation=1.0, rate_in_hz=-0.1)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation=1.0, rate_in_hz=0.0)

        current_activation = 1.0
        for rate in [1.0, 10.0, 100.0, 1000.0]:
            next_value = strategy.get_next_value(current_activation=current_activation, rate_in_hz=rate)
            self.assertAlmostEqual(current_activation - (strategy.slope / rate), next_value)

        for current_activation in [0.1, 0.5, 0.75, 0.9, 1.0]:
            next_value = strategy.get_next_value(current_activation=current_activation, rate_in_hz=1.0)
            self.assertAlmostEqual(current_activation - strategy.slope, next_value)

        # Verify that MIN_ACTIVATION is enforced
        next_value = strategy.get_next_value(current_activation=MIN_ACTIVATION, rate_in_hz=1.0)
        self.assertAlmostEqual(MIN_ACTIVATION, next_value)


class SigmoidDecayStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        strategy = SigmoidDecayStrategy(rate_multiplier=0.1)
        self.assertEqual(strategy.rate_multiplier, 0.1)

        strategy = SigmoidDecayStrategy()
        self.assertEqual(strategy.rate_multiplier, 1.0)

        with self.assertRaises(ValueError):
            SigmoidDecayStrategy(rate_multiplier=0.0)

        with self.assertRaises(ValueError):
            SigmoidDecayStrategy(rate_multiplier=-1.0)

    def test_get_next_value(self):
        # Verify expected value after decrease starting at activation = 0.5
        strategy = SigmoidDecayStrategy()

        current_activation = 0.5

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.268941, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidDecayStrategy(2.0)

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.119203, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidDecayStrategy(0.5)

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.377541, places=5)

        # Verify expected value after decrease starting at activation = MIN_ACTIVATION is
        # still approximately MIN_ACTIVATION
        strategy = SigmoidDecayStrategy()

        next_value = strategy.get_next_value(MIN_ACTIVATION, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, MIN_ACTIVATION, places=5)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation, rate_in_hz=0.0)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation, rate_in_hz=-1.0)


class SigmoidExciteStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_get_next_value(self):
        # Verify expected value after decrease starting a activation = 0.5
        strategy = SigmoidExciteStrategy()

        current_activation = 0.5

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.731059, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidExciteStrategy(2.0)

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.880797, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidExciteStrategy(0.5)

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.622459, places=5)

        # Verify expected value after excitation starting at activation = MAX_ACTIVATION is
        # still approximately MAX_ACTIVATION
        strategy = SigmoidExciteStrategy()

        next_value = strategy.get_next_value(MAX_ACTIVATION, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, MAX_ACTIVATION, places=5)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation, rate_in_hz=0.0)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation, rate_in_hz=-1.0)
