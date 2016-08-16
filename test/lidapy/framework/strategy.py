from nose.tools import raises
import unittest

from math import fabs

from lidapy.framework.strategy import LinearDecayStrategy, LinearExciteStrategy, SigmoidDecayStrategy, SigmoidExciteStrategy, MIN_ACTIVATION, MAX_ACTIVATION, EPSILON_MIN


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
        current_activation = 0.0
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        assert(new_activation == current_activation + 0.01)

        # Verify that increasing activation beyond MAX_ACTIVATION results in MAX_ACTIVATION
        strategy = LinearExciteStrategy(0.01)
        current_activation = MAX_ACTIVATION
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        assert (new_activation == MAX_ACTIVATION)

    @raises(ValueError)
    def test_exception(self):
        strategy = LinearExciteStrategy(0.01)
        current_activation = MIN_ACTIVATION
        rate_in_hz = 0

        strategy.apply(current_activation, rate_in_hz)


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
        current_activation = MAX_ACTIVATION
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        assert(new_activation == current_activation - 0.01)

        # Verify that decreasing activation beyond MIN_ACTIVATION results in MIN_ACTIVATION
        strategy = LinearDecayStrategy(0.01)
        current_activation = MIN_ACTIVATION
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        assert (new_activation == MIN_ACTIVATION)

    # Verify ValueError raised when rate_in_hz < 0
    @raises(ValueError)
    def test_exception(self):
        strategy = LinearDecayStrategy(0.01)
        current_activation = MAX_ACTIVATION
        rate_in_hz = 0

        strategy.apply(current_activation, rate_in_hz)


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
        current_activation = 0.5
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        new_activation_rnded = "{0:.6f}".format(new_activation)
        assert (new_activation_rnded == str(0.268941))

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidDecayStrategy(2.0)
        current_activation = 0.5
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        new_activation_rnded = "{0:.6f}".format(new_activation)
        assert (new_activation_rnded == str(0.119203))

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidDecayStrategy(0.5)
        current_activation = 0.5
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        new_activation_rnded = "{0:.6f}".format(new_activation)
        assert (new_activation_rnded == str(0.377541))

        # Verify expected value after decrease starting at activation = MIN_ACTIVATION is
        # still approximately MIN_ACTIVATION
        strategy = SigmoidDecayStrategy()
        current_activation = MIN_ACTIVATION
        rate_in_hz = 1

        new_activation = strategy.apply(current_activation, rate_in_hz)
        assert (fabs(MIN_ACTIVATION - new_activation) <= EPSILON_MIN)

    # Verify ValueError raised when rate_in_hz < 0
    @raises(ValueError)
    def test_rate_in_hz_exception(self):
        strategy = SigmoidDecayStrategy()
        current_activation = MAX_ACTIVATION
        rate_in_hz = 0

        strategy.apply(current_activation, rate_in_hz)

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
        current_activation = 0.5
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        new_activation_rnded = "{0:.6f}".format(new_activation)
        assert (new_activation_rnded == str(0.731059))

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidExciteStrategy(2.0)
        current_activation = 0.5
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        new_activation_rnded = "{0:.6f}".format(new_activation)
        assert (new_activation_rnded == str(0.880797))

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidExciteStrategy(0.5)
        current_activation = 0.5
        rate_in_hz = 1.0

        new_activation = strategy.apply(current_activation, rate_in_hz)
        new_activation_rnded = "{0:.6f}".format(new_activation)
        assert (new_activation_rnded == str(0.622459))

        # Verify expected value after excitation starting at activation = MAX_ACTIVATION is
        # still approximately MAX_ACTIVATION
        strategy = SigmoidExciteStrategy()
        current_activation = MAX_ACTIVATION
        rate_in_hz = 1

        new_activation = strategy.apply(current_activation, rate_in_hz)
        assert (fabs(MAX_ACTIVATION - new_activation) <= EPSILON_MIN)

    # Verify ValueError raised when rate_in_hz < 0
    @raises(ValueError)
    def test_rate_in_hz_exception(self):
        strategy = SigmoidExciteStrategy()
        current_activation = MAX_ACTIVATION
        rate_in_hz = 0

        strategy.apply(current_activation, rate_in_hz)

    # Verify ValueError raised when rate_multiplier <= 0
    @raises(ValueError)
    def test_rate_multiplier_exception(self):
        SigmoidExciteStrategy(0)

