from __future__ import print_function

from abc import ABCMeta, abstractmethod
from datetime import datetime
from sys import stdout, stderr

from rospy import logdebug, loginfo, logwarn, logerr, logfatal

DEBUG = 0
INFO = 1
WARN = 2
ERROR = 3
FATAL = 4

DEFAULT_LOG_LEVEL = INFO


class AbstractLogger(object):
    __metaclass__ = ABCMeta

    def __init__(self, log_level=DEFAULT_LOG_LEVEL):
        self.log_level = log_level

    @abstractmethod
    def debug(self, msg):
        pass

    @abstractmethod
    def info(self, msg):
        pass

    @abstractmethod
    def warn(self, msg):
        pass

    @abstractmethod
    def error(self, msg):
        pass

    @abstractmethod
    def fatal(self, msg):
        pass

    @property
    def log_level(self):
        return self._log_level

    @log_level.setter
    def log_level(self, value):
        if type(value) is not int:
            raise ("Invalid log level: {}".format(value))

        self._log_level = value


class ConsoleLogger(AbstractLogger):
    MSG_FORMAT = "{timestamp} [{log_level}] {message}"

    def __init__(self, log_level=DEFAULT_LOG_LEVEL):
        super(ConsoleLogger, self).__init__(log_level)

    def debug(self, msg):
        if DEBUG >= self.log_level:
            print(self._get_formatted_msg(msg, "DEBUG"), file=stdout)

    def info(self, msg):
        if INFO >= self.log_level:
            print(self._get_formatted_msg(msg, "INFO"), file=stdout)

    def warn(self, msg):
        if WARN >= self.log_level:
            print(self._get_formatted_msg(msg, "WARN"), file=stderr)

    def error(self, msg):
        if ERROR >= self.log_level:
            print(self._get_formatted_msg(msg, "ERROR"), file=stderr)

    def fatal(self, msg):
        if FATAL >= self.log_level:
            print(self._get_formatted_msg(msg, "FATAL"), file=stderr)

    def _get_formatted_msg(self, msg, level):
        return ConsoleLogger.MSG_FORMAT.format(timestamp=datetime.now(),
                                               log_level=str.upper(level),
                                               message=msg)


class RosLogger(AbstractLogger):
    def __init__(self, log_level=DEFAULT_LOG_LEVEL):
        super(RosLogger, self).__init__(log_level)

    def debug(self, msg):
        logdebug(msg)

    def info(self, msg):
        loginfo(msg)

    def warn(self, msg):
        logwarn(msg)

    def error(self, msg):
        logerr(msg)

    def fatal(self, msg):
        logfatal(msg)
