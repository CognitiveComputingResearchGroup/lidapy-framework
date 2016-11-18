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

    def test_change(self):
        # Verify linear increase in activation
        strategy = LinearExciteStrategy(0.01)

        initial_activation = 0.0
        cc = CognitiveContent("value")
        cc.activation = initial_activation

        strategy.apply(cc, rate_in_hz=1.0)
        self.assertEqual(cc.activation, initial_activation + 0.01)

        # Verify that increasing activation beyond MAX_ACTIVATION results in MAX_ACTIVATION
        strategy = LinearExciteStrategy(0.01)
        cc.activation = MAX_ACTIVATION

        strategy.apply(cc, rate_in_hz=1.0)
        self.assertEqual(cc.activation, MAX_ACTIVATION)

    @raises(ValueError)
    def test_exception(self):
        strategy = LinearExciteStrategy(0.01)

        cc = CognitiveContent("value")
        cc.activation = MIN_ACTIVATION

        strategy.apply(cc, rate_in_hz=0.0)


class LinearDecayStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_change(self):
        # Verify linear decrease in activation
        strategy = LinearDecayStrategy(0.01)

        cc = CognitiveContent("value")
        cc.activation = MAX_ACTIVATION

        strategy.apply(cc, rate_in_hz=1.0)
        self.assertEqual(cc.activation, MAX_ACTIVATION - 0.01)

        # Verify that decreasing activation beyond MIN_ACTIVATION results in MIN_ACTIVATION
        strategy = LinearDecayStrategy(0.01)

        cc.activation = MIN_ACTIVATION

        strategy.apply(cc, rate_in_hz=1.0)
        self.assertEqual(cc.activation, MIN_ACTIVATION)

    # Verify ValueError raised when rate_in_hz < 0
    @raises(ValueError)
    def test_exception(self):
        strategy = LinearDecayStrategy(0.01)

        cc = CognitiveContent("value")
        cc.activation = MAX_ACTIVATION

        rate_in_hz = 0

        strategy.apply(cc, rate_in_hz)


class SigmoidDecayStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_change(self):
        # Verify expected value after decrease starting a activation = 0.5
        strategy = SigmoidDecayStrategy()

        cc = CognitiveContent("value")
        cc.activation = 0.5

        strategy.apply(cc, rate_in_hz=1.0)
        self.assertAlmostEqual(cc.activation, 0.268941, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidDecayStrategy(2.0)

        cc = CognitiveContent("value")
        cc.activation = 0.5

        strategy.apply(cc, rate_in_hz=1.0)
        self.assertAlmostEqual(cc.activation, 0.119203, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidDecayStrategy(0.5)

        cc = CognitiveContent("value")
        cc.activation = 0.5

        strategy.apply(cc, rate_in_hz=1.0)
        self.assertAlmostEqual(cc.activation, 0.377541, places=5)

        # Verify expected value after decrease starting at activation = MIN_ACTIVATION is
        # still approximately MIN_ACTIVATION
        strategy = SigmoidDecayStrategy()

        cc = CognitiveContent("value")
        cc.activation = MIN_ACTIVATION

        strategy.apply(cc, rate_in_hz=1.0)
        self.assertAlmostEqual(cc.activation, MIN_ACTIVATION, places=5)

    # Verify ValueError raised when rate_in_hz < 0
    @raises(ValueError)
    def test_rate_in_hz_exception(self):
        strategy = SigmoidDecayStrategy()

        cc = CognitiveContent("value")
        cc.activation = MAX_ACTIVATION

        rate_in_hz = 0

        strategy.apply(cc.activation, rate_in_hz)

    # Verify ValueError raised when rate_multiplier <= 0
    @raises(ValueError)
    def test_rate_multiplier_exception(self):
        SigmoidDecayStrategy(0)


class SigmoidExciteStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_change(self):
        # Verify expected value after decrease starting a activation = 0.5
        strategy = SigmoidExciteStrategy()

        cc = CognitiveContent("value")
        cc.activation = 0.5

        rate_in_hz = 1.0

        strategy.apply(cc, rate_in_hz)
        self.assertAlmostEqual(cc.activation, 0.731059, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidExciteStrategy(2.0)

        cc = CognitiveContent("value")
        cc.activation = 0.5

        rate_in_hz = 1.0

        strategy.apply(cc, rate_in_hz)
        self.assertAlmostEqual(cc.activation, 0.880797, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidExciteStrategy(0.5)

        cc = CognitiveContent("value")
        cc.activation = 0.5

        rate_in_hz = 1.0

        strategy.apply(cc, rate_in_hz)
        self.assertAlmostEqual(cc.activation, 0.622459, places=5)

        # Verify expected value after excitation starting at activation = MAX_ACTIVATION is
        # still approximately MAX_ACTIVATION
        strategy = SigmoidExciteStrategy()

        cc = CognitiveContent("value")
        cc.activation = MAX_ACTIVATION

        rate_in_hz = 1.0

        strategy.apply(cc, rate_in_hz)
        self.assertAlmostEqual(cc.activation, MAX_ACTIVATION, places=5)

    # Verify ValueError raised when rate_in_hz < 0
    @raises(ValueError)
    def test_rate_in_hz_exception(self):
        strategy = SigmoidExciteStrategy()

        cc = CognitiveContent("value")
        cc.activation = MAX_ACTIVATION

        rate_in_hz = 0

        strategy.apply(cc, rate_in_hz)

    # Verify ValueError raised when rate_multiplier <= 0
    @raises(ValueError)
    def test_rate_multiplier_exception(self):
        SigmoidExciteStrategy(0)
