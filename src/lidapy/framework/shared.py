from lidapy.util.meta import Singleton


class FrameworkObject(object):
    def __init__(self):
        self._dependencies = {"config": FrameworkDependency("config"),
                              "logger": FrameworkDependency("logger"),
                              "ipc_proxy": FrameworkDependency("ipc_proxy")}

    @property
    def logger(self):
        logger = self._dependencies["logger"].resolve()
        return logger

    @property
    def ipc_proxy(self):
        ipc_proxy = self._dependencies["ipc_proxy"].resolve()
        return ipc_proxy

    @property
    def config(self):
        config = self._dependencies["config"].resolve()
        return config


class FrameworkDependency(object):
    def __init__(self, name):
        self.name = name

        self._fds = FrameworkDependencyService()

    def resolve(self):
        return self._fds[self.name]

    def is_satisfied(self):
        return self._fds.has(self.name)

        # def __getattr__(self, attr):
        #     resolved_attr = self._depend.__getattribute__(attr)
        #     if callable(resolved_attr):
        #
        #         def wrapped(*args, **kwargs):
        #             result = resolved_attr(*args, **kwargs)
        #             if result == self._depend:
        #                 return self
        #             return result
        #         return wrapped
        #
        #     else:
        #         return resolved_attr


class FrameworkDependencyService(object):
    __metaclass__ = Singleton

    def __init__(self, allow_overrides=False):
        self.allow_overrides = allow_overrides

        self._dependencies = {}

    def __setitem__(self, key, dependency):
        if self._dependencies.has_key(key):
            if not self.allow_overrides:
                raise Exception("Dependency overrides not allowed.")
        else:
            self._dependencies[key] = dependency

    def __getitem__(self, key):
        dependency = self._dependencies.get(key, None)
        if dependency is None:
            raise Exception("Dependency for key {} does not exist".format(key))

        return dependency

    def __len__(self):
        return len(self._dependencies)

    def has(self, key):
        return self._dependencies.has_key(key)


class Activatable(object):
    def __init__(self, initial_activation=0.0, initial_incentive_salience=0.0, initial_base_level_activation=0.0):
        self._activation = 0.0
        self._incentive_salience = 0.0
        self._base_level_activation = 0.0

        self.activation = initial_activation
        self.incentive_salience = initial_incentive_salience
        self.base_level_activation = initial_base_level_activation

    @property
    def activation(self):
        return self._activation

    @activation.setter
    def activation(self, activation):
        if activation < 0.0:
            self._activation = 0.0
        elif activation > 1.0:
            self._activation = 1.0
        else:
            self._activation = activation

    @property
    def base_level_activation(self):
        return self._base_level_activation

    @base_level_activation.setter
    def base_level_activation(self, base_level_activation):
        if base_level_activation < 0.0:
            self._base_level_activation = 0.0
        elif base_level_activation > 1.0:
            self._base_level_activation = 1.0
        else:
            self._base_level_activation = base_level_activation

    @property
    def incentive_salience(self):
        return self._incentive_salience

    @incentive_salience.setter
    def incentive_salience(self, incentive_salience):
        if incentive_salience < 0.0:
            self._incentive_salience = 0.0
        elif incentive_salience > 1.0:
            self._incentive_salience = 1.0
        else:
            self._incentive_salience = incentive_salience


class CognitiveContent(Activatable):
    def __init__(self, value):
        super(CognitiveContent, self).__init__()

        self.value = value

    def __eq__(self, other):
        if other is None:
            return False

        return self.value == other.value


class CognitiveContentStructure(object):
    def __init__(self):
        self._content_list = []

    def __add__(self, other):
        self._content_list += other

    def __len__(self):
        return len(self._content_list)

    def __iter__(self):
        return CognitiveContentStructureIterator(self._content_list)

    def insert(self, content):
        self._content_list.append(content)

    def remove(self, content):
        self._content_list.remove(content)


class CognitiveContentStructureIterator(object):
    def __init__(self, content):
        self.content = content
        self.index = 0

    def next(self):
        try:
            next_content = self.content[self.index]
        except IndexError:
            raise StopIteration()

        self.index += 1
        return next_content

    def __iter__(self):
        return self
