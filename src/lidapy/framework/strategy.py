from math import log, exp

MIN_ACTIVATION = 0.0
MAX_ACTIVATION = 1.0
EPSILON_MIN = 1.0e-5


class LinearDecayStrategy(object):
    def __init__(self, slope):
        self.slope = slope

    def apply(self, current_activation, rate_in_hz):
        if rate_in_hz <= 0:
            raise ValueError("Invalid value for rate_in_hz ({})".format(rate_in_hz))

        new_activation = current_activation - self.slope / rate_in_hz

        if new_activation < MIN_ACTIVATION:
            new_activation = MIN_ACTIVATION

        return new_activation


class LinearExciteStrategy(object):
    def __init__(self, slope):
        self.slope = slope

    def apply(self, current_activation, rate_in_hz):
        if rate_in_hz <= 0:
            raise ValueError("Invalid value for rate_in_hz ({})".format(rate_in_hz))

        new_activation = current_activation + self.slope / rate_in_hz

        if new_activation > MAX_ACTIVATION:
            new_activation = MAX_ACTIVATION

        return new_activation


class SigmoidHelper(object):
    @staticmethod
    def get_time_from_activation(current_activation):
        if current_activation >= 1.0:
            current_activation = 1.0 - EPSILON_MIN
        elif current_activation <= 0.0:
            current_activation = EPSILON_MIN

        return -log(1.0 / current_activation - 1)


class SigmoidDecayStrategy(object):
    def __init__(self, rate_multiplier=1.0):
        if rate_multiplier <= 0.0:
            raise ValueError("Invalid value for rate_multiplier (must be > 0).")

        self.rate_multiplier = rate_multiplier

    def apply(self, current_activation, rate_in_hz):
        if rate_in_hz <= 0:
            raise ValueError("Invalid value for rate_in_hz (must be > 0)".format(rate_in_hz))

        t = SigmoidHelper.get_time_from_activation(current_activation) - self.rate_multiplier / rate_in_hz

        new_activation = 1.0 / (1.0 + exp(-t))
        if new_activation < MIN_ACTIVATION:
            new_activation = MIN_ACTIVATION

        return new_activation


class SigmoidExciteStrategy(object):
    def __init__(self, rate_multiplier=1.0):
        if rate_multiplier <= 0.0:
            raise ValueError("Invalid value for rate_multiplier (must be > 0).")

        self.rate_multiplier = rate_multiplier

    def apply(self, current_activation, rate_in_hz):
        if rate_in_hz <= 0:
            raise ValueError("Invalid value for rate_in_hz (must be > 0)".format(rate_in_hz))

        t = SigmoidHelper.get_time_from_activation(current_activation) + self.rate_multiplier / rate_in_hz

        new_activation = 1.0 / (1 + exp(-t))
        if new_activation > MAX_ACTIVATION:
            new_activation = MAX_ACTIVATION

        return new_activation
