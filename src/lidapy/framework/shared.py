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
