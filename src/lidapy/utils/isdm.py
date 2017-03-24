from math import pi, sin, cos, atan
import platform
import shutil
try:
    import msgpack as pickle
    print("msgpack imported")
except ImportError:
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
same_vector_distance_threshold = 100


class ModularDimension(object):
    global TWO_PI

    __slots__ = ['mag', 'value']

    def __init__(self, value):
        self.mag = 1
        self.value = value

    @property
    def theta(self):
        return self.value * del_theta

    @theta.setter
    def theta(self, theta_):
        self.value = round(theta_/del_theta) % _axis_length

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
v = ModularDimension(0)
r = ModularDimension(7)
t = ModularDimension(13)
print((v+t+r).value)

sin = dict([(key, np.sin(key*del_theta)) for key in range(_axis_length)])
cos = dict([(key, np.cos(key*del_theta)) for key in range(_axis_length)])

def add_two_dimensions(value1, magnitude1, value2, magnitude2):
    # x and y components are flipped for visualizing the vectors on a clock style circle with 0 at the top
    x = magnitude1*cos[value1] + magnitude2*cos[value2]
    y = magnitude1*sin[value1] + magnitude2*sin[value2]

    if x == 0:
        if y > 0:
            result = (TWO_PI/4)
        else:
            result = (TWO_PI*(3.0/4))
    else:
        result = atan(abs(y/x))

    if y < 0 < x:
        result = TWO_PI - result
    elif y < 0 and x < 0:
        result = TWO_PI / 2 + result
    elif x < 0 < y:
        result = TWO_PI / 2 - result

    if x == 0 and y == 0:
        # insert the chance logic
        result = atan(sin[value1]/cos[value1]) + TWO_PI / 4

    magnitude = (value1**2+value2**2)**.5

    return (round(result/del_theta) % _axis_length, magnitude)

import itertools

for a,b in itertools.combinations(range(16), 2):
    print(a, b, add_two_dimensions(a, 1, b, 1)[0])


class MCRVector(object):

    __slots__ = ['_dims', '_factor', '_magnitudes_']
    def __init__(self, dims, factor=1, _mag=np.array([1]*ndim)):
        if len(dims) != ndim:
            raise ValueError
        self._dims = dims
        self._factor = factor
        self._magnitudes_ = _mag

    @property
    def _magnitudes(self):
        magnitudes = self._magnitudes_
        self._magnitudes_ = [1]*ndim
        return magnitudes

    @classmethod
    def random_vector(cls):
        _dims = np.random.random_integers(r_min, r_max, ndim)
        return cls(_dims)

    def __len__(self):
        return len(self._dims)

    def __invert__(self):
        dims_copy = [(_axis_length-dim) % _axis_length for dim in self.dims]
        return MCRVector(np.array(dims_copy))

    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return MCRVector(self.dims.copy(), factor=self.factor*other)
        else:
            return MCRVector(np.apply_along_axis(lambda dim: dim % _axis_length, 0, self.dims+other.dims))

    def __add__(self, other):
        self_magnitudes = self._magnitudes
        other_magnitudes = other._magnitudes
        addition_result = [add_two_dimensions(*param_values) for param_values in zip(self.dims,
                                                                                     self_magnitudes*self.factor,
                                                                                     other.dims,
                                                                                     other_magnitudes*other.factor)]
        return MCRVector(np.array([i[0] for i in addition_result]), _mag=[i[1] for i in addition_result])

    def __getitem__(self, item):
        return self._dims[item]

    def distance(self, other):
        return sum([min((self_dim-other_dim) % _axis_length,
                        (other_dim-self_dim) % _axis_length) for self_dim, other_dim in zip(self.dims, other.dims)])

    @property
    def dims(self):
        return self._dims

    @property
    def factor(self):
        return self._factor


class HardLocation(MCRVector):

    __slots__ = ['_name', '_dims', '_factor', '_magnitudes_']

    def __init__(self, name):
        _vector = MCRVector.random_vector().dims.copy()
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
            counters[i][dim] += 1
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
            created = True
        except FileExistsError:
            print("This location already stores a memory. Run from a different location")
            created = False
        self._created = created

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

    def read(self, address, prev_distances=[]):  # TODO fix default []
        if len(prev_distances) > 2:
            if prev_distances[-1] < same_vector_distance_threshold and len(prev_distances) < 100:
                print(prev_distances)
                print("read by convergence")
                return address

            if np.mean([prev_distances[i]-prev_distances[i-1] for i in range(1, len(prev_distances))]) > 0:
                raise Exception("cannot be read")

        hard_locations_in_radius = [location for location in self.hard_locations
                                    if address.distance(location) < self.access_sphere_radius]

        if len(hard_locations_in_radius)<0:
            raise Exception("cannot be read")
        # adding the counters of locations in radius
        total = HardLocation.get_empty_counter()
        for location in hard_locations_in_radius:
            total = location.add_counters(total)

        # creating the word from max of counters
        word = list()
        for counter in total:
            word.append(np.argmax(counter))
        word = MCRVector(np.array(word))

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
    print(v1.distance(ram))
    print(v1.distance(eat))
    v1 = pam.read(v1)
    print(v1.distance(cake))
    print(v1.distance(sita))
    print(cake.distance(sita))
