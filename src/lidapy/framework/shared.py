from lidapy.util.meta import Singleton


class FrameworkObject(object):
    def __init__(self):
        self._dependencies = {"config": FrameworkDependency("config"),
                              "logger": FrameworkDependency("logger"),
                              "ipc_proxy": FrameworkDependency("ipc_proxy")}

    @property
    def logger(self):
        return self._dependencies["logger"].resolve()

    @property
    def ipc_proxy(self):
        return self._dependencies["ipc_proxy"].resolve()

    @property
    def config(self):
        return self._dependencies["config"].resolve()


class FrameworkDependency(object):
    def __init__(self, name):
        self.name = name

        self._fds = FrameworkDependencyService()

    def resolve(self):
        return self._fds[self.name]

    def is_satisfied(self):
        return self._fds.has(self.name)


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
    def __init__(self, initial_activation=0.0, initial_incentive_salience=0.0, initial_base_level_activation=0.0,
                 removal_threshold=0.0):
        self._activation = 0.0
        self._incentive_salience = 0.0
        self._base_level_activation = 0.0
        self._removal_threshold = 0.0

        self.activation = initial_activation
        self.incentive_salience = initial_incentive_salience
        self.base_level_activation = initial_base_level_activation
        self.removal_threshold = removal_threshold

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

    @property
    def removal_threshold(self):
        return self._activation

    @removal_threshold.setter
    def removal_threshold(self, removal_threshold):
        if removal_threshold < 0.0:
            self._removal_threshold = 0.0
        elif removal_threshold > 1.0:
            self._removal_threshold = 1.0
        else:
            self._removal_threshold = removal_threshold


class CognitiveContent(Activatable):
    def __init__(self, value):
        super(CognitiveContent, self).__init__()

        # Prevent nested cognitive content
        if isinstance(value, CognitiveContent):
            self.copy(value)
        else:
            self.value = value

    def copy(self, other):
        self.value = other.value
        self.activation = other.activation
        self.incentive_salience = other.incentive_salience
        self.base_level_activation = other.base_level_activation
        self.removal_threshold = other.removal_threshold

    def __eq__(self, other):
        if other is None:
            return False

        return self.value == other.value


class CognitiveContentStructure(object):
    def __init__(self, content_list=None):

        try:
            if content_list is None:
                self._content_list = list()
            else:
                self._content_list = list(content_list)
        except TypeError:
            raise TypeError("CognitiveContentStructure: content_list must be of iterable type")

        for value in self._content_list:
            if not issubclass(value.__class__, CognitiveContent):
                raise TypeError("CognitiveContentStructure: all elements in content_list must be of iterable type")

    def __add__(self, other):
        temp = CognitiveContentStructure()

        if issubclass(other.__class__, CognitiveContentStructure):
            temp._content_list += self._content_list + other._content_list
        else:
            try:
                temp._content_list += self._content_list + other
            except TypeError:
                raise TypeError("Add operand must be either a CognitiveContentStructure or an iterable")

        return temp

    def __radd__(self, other):
        return self.__add__(other)

    def __iadd__(self, other):
        if issubclass(other.__class__, CognitiveContentStructure):
            self._content_list += other._content_list
        else:
            try:
                self._content_list += other
            except TypeError:
                raise TypeError("Add operand must be either a CognitiveContentStructure or an iterable")

        return self

    def __len__(self):
        return len(self._content_list)

    def __iter__(self):
        return CognitiveContentStructureIterator(self._content_list)

    def insert(self, content):
        self._content_list.append(content)

    def remove(self, content):
        try:
            self._content_list.remove(content)
        except:
            # Ignore attempts to remove non-existent content
            pass

    def remove_all_matches(self, condition):
        match_list = []
        for cc in self._content_list:
            if condition(cc):
                match_list.append(cc)

        for cc in match_list:
            self._content_list.remove(cc)

    def apply(self, func):
        for cc in self._content_list:
            func(cc)


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
