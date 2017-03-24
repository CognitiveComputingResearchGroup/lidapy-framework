from math import pi, sin, cos, atan
import platform
import shutil
import pickle
import os
import numpy as np
from functools import lru_cache
'''
if platform.python_version_tuple()[0] == '2':
    import repoze.lru.lru_cache as lru_cache
else:
    import functools.lru_cache as lru_cache
'''
r = (0, 15)
_axis_length = r[1]-r[0]+1
r_max = r[1]
r_min = r[0]
TWO_PI = 2*pi
del_theta = TWO_PI/_axis_length
ndim = 1000
_memory_location = 'pam'


class ModularDimension(object):
    global TWO_PI

    __slots__ = ['mag', '_theta']

    def __init__(self, value):
        self.mag = 1
        self._theta = 0
        self.value = value

    @property
    def value(self):
        val = self._theta/del_theta
        return int(round(val)) % _axis_length

    @value.setter
    def value(self, value_):
        self._theta = value_ * del_theta

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, theta_):
        self. _theta = theta_

    def __add__(self, other):
        result = ModularDimension(0)

        x = other.mag * sin(other.theta) + self.mag * sin(self.theta)
        y = other.mag * cos(other.theta) + self.mag * cos(self.theta)
        if y == 0:
            if x > 0:
                result.theta = (TWO_PI/4)
            else:
                result.theta = (TWO_PI*(3.0/4))
        else:
            result.theta = atan(x/y)

        if x == 0 and y == 0:
            # insert the chance logic
            result.theta = self.theta + TWO_PI / 4

        result.mag = (y**2+x**2)**0.5
        return result

    def __sub__(self, other):
        return ModularDimension(min([(self.value-other.value) % _axis_length,
                                     (other.value-self.value) % _axis_length]))

    def __mul__(self, other):
        return ModularDimension((self.value+other.value) % _axis_length)

    def __invert__(self):
        return ModularDimension((r_max+1-self.value) % _axis_length)

import itertools

for a,b in itertools.combinations(range(15), 2):
    print(a, b, (ModularDimension(a)+ModularDimension(b)).value)

class MCRVector(object):

    def __init__(self, dims):
        if len(dims) != ndim:
            raise ValueError
        self._dims = [ModularDimension(i) for i in dims]

    @classmethod
    def random_vector(cls):
        _dims = np.random.random_integers(r_min, r_max, ndim)
        return cls(_dims)

    def __len__(self):
        return len(self._dims)

    def __invert__(self):
        dims_copy = [(~dim).value for dim in self.dims]

        return MCRVector(dims_copy)

    def __mul__(self, other):
        return MCRVector([(self[i]*other[i]).value for i in range(len(other))])

    def __add__(self, other):
        return MCRVector([(self[i]+other[i]).value for i in range(len(other))])

    def __getitem__(self, item):
        return self._dims[item]

    def distance(self, other):
        return sum([(self[i]-other[i]).value for i in range(len(other))])

    @property
    def dims(self):
        return self._dims


class HardLocation(MCRVector):

    __slots__ = ['_counter', '_dims']

    def __init__(self, name):
        _vector = [dim.value for dim in MCRVector.random_vector().dims]
        super(HardLocation, self).__init__(_vector)
        self._name = name

    @staticmethod
    def get_empty_counter():
        return [np.array([0]*_axis_length) for _ in range(ndim)]

    def create_counters(self):
        empty_counters = HardLocation.get_empty_counter()
        IntegerSDM.store_counters(self._name, empty_counters)
        return empty_counters

    def write(self, word):
        counters = self.counters
        for i, dim in enumerate(word.dims):
            try:
                counters[i][dim.value] += 1
            except IndexError:
                print(dim.value)
        self._update_counters(counters)

    def add_counters(self, counters):
        resultant_counters = []
        self_counters = self.counters
        for i, counter_dim in enumerate(counters):
            resultant_counters.append(counters[i]+self_counters[i])

        return resultant_counters

    def _update_counters(self, counters):
        IntegerSDM.store_counters(self._name, counters)
        self.counters

    @property
    def counters(self):
        counters = IntegerSDM.retrieve_counters(self._name)
        if counters is None:
            counters = self.create_counters()
        return counters

    @property
    def name(self):
        return self._name


class IntegerSDM(object):
    def __init__(self, n_hard_locations):
        self.hard_locations = [HardLocation(name='location'+str(i)) for i in range(n_hard_locations)]
        if _axis_length % 2 != 0:
            raise Exception('r should be even')
        # phi_inv = scipy.stats.norm.ppf(0.001) the calculation used below
        phi_inv = -3.0902323061678132 # for p=0.0001 phi_inv(p) = -3.0902...
        r_ = _axis_length
        self.access_sphere_radius = ((ndim*(r_**2+8)/48)**.5)*phi_inv+((ndim*r_)/4)
        try:
            os.mkdir(_memory_location)
            self._created = True
        except FileExistsError:
            print("This location already stores a memory. Run from a different location")
            self._created = False

    @staticmethod
    @lru_cache(maxsize=100)
    def retrieve_counters(hard_location_name):
        try:
            mem_file = open(os.path.join(_memory_location, hard_location_name), 'rb')
        except FileNotFoundError:
            return None
        return pickle.load(mem_file, -1)

    @staticmethod
    def store_counters(hard_location_name, counters):
        mem_file = open(os.path.join(_memory_location, hard_location_name), 'wb')
        pickle.dump(counters, mem_file, -1)

    def read(self, address, prev_distances=[8000]):  # TODO fix default []
        if len(prev_distances) > 100 or prev_distances[-1] < 100:
            return address

        hard_locations_in_radius = [location for location in self.hard_locations
                                    if address.distance(location) < self.access_sphere_radius]

        # adding the counters of locations in radius
        total = HardLocation.get_empty_counter()
        for location in hard_locations_in_radius:
            total = location.add_counters(total)

        # creating the word from max of counters
        word = list()
        for counter in total:
            word.append(np.argmax(counter))
        word = MCRVector(word)

        prev_distances.append(address.distance(word))

        return self.read(word, prev_distances)

    def write(self, word):
        hard_locations_in_radius = [location for location in self.hard_locations
                                    if word.distance(location) < self.access_sphere_radius]
        if len(hard_locations_in_radius) < 1:
            raise Exception("cannot be written")
        for location in hard_locations_in_radius:
            location.write(word)

    def __del__(self):
        if self._created:
            shutil.rmtree(_memory_location)


if __name__ == '__main__':
    pam = IntegerSDM(1000)
    cake = MCRVector.random_vector()
    ram = MCRVector.random_vector()
    apple = MCRVector.random_vector()
    sita = MCRVector.random_vector()
    eat = ram*cake + sita*apple

    print(eat.distance(ram))

    pam.write(cake)
    pam.write(ram)
    pam.write(apple)
    pam.write(sita)

    v1 = eat*(~ram)
    v1 = pam.read(v1)
    print(v1.distance(cake))
    print(v1.distance(sita))
    print(cake.distance(sita))
