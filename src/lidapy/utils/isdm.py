from math import pi, sin, cos, atan
import numpy as np

r = (0, 15)
_axis_length = r[1]-r[0]+1
r_max = r[1]
r_min = r[0]
TWO_PI = 2*pi
del_theta = TWO_PI/_axis_length
ndim = 1000

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
    def theta(self, theta):
        self.value = int(theta/self.del_theta)

    def __add__(self, other):
        result = ModularDimension(0)
        y = other.mag * sin(other.theta) + self.mag * sin(self.theta)
        x = other.mag * cos(other.theta) + self.mag * cos(self.theta)
        if x == 0:
            if y > 0:
                result.theta = (TWO_PI / 4)
            else:
                result.theta = (TWO_PI * 3 / 4)
        else:
            result.theta = atan(abs(y / x))

        if y < 0 < x:
            result.theta = TWO_PI - result.theta
        elif y < 0 and x < 0:
            result.theta = TWO_PI / 2 + result.theta
        elif x < 0 < y:
            result.theta = TWO_PI / 2 - result.theta

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


class MCRVector(object):

    def __init__(self, dims):
        if len(dims) != ndim:
            raise ValueError
        self._dims = np.array(dims, dtype=np.uint8)

    def __len__(self):
        return len(self._dims)

    def __invert__(self):
        dims_copy = self._dims.copy()
        np.apply_along_axis(lambda dim: r_max+1-dim % _axis_length, 0, dims_copy)
        return dims_copy

    @staticmethod
    def dim_product(dim1, dim2):
        return dim1+dim2 % _axis_length

    @staticmethod
    def dim_sum(dim1, dim2):
        dim_one = ModularDimension(dim1)
        dim_two = ModularDimension(dim2)
        result = dim_one+dim_two
        return result.value

    @staticmethod
    def dim_sub(dim1, dim2):
        return (min([(dim1-dim2) % _axis_length,
                     (dim1-dim1) % _axis_length]))

    def __mul__(self, other):
        return MCRVector([MCRVector.dim_product(self[i], other[i]) for i in range(len(other))])

    def __add__(self, other):  # Catch - every add operation resets the magnitude of bits. Not so in [Snaider, 2012]
        return MCRVector([MCRVector.dim_sum(self[i], other[i]) for i in range(len(other))])

    def distance(self, other):
        return sum([MCRVector.dim_sub(self[i], other[i]) for i in range(len(other))])

    def __getitem__(self, item):
        return self._dims[item]

    @classmethod
    def randomvector(cls):
        _dims = np.random.random_integers(r_min, r_max, ndim)
        return cls(_dims)


class HardLocation(MCRVector):

    __slots__ = ['_counter', '_dims']

    def __init__(self):
        _vector = [dim for dim in MCRVector.randomvector()._dims]
        super(HardLocation, self).__init__(_vector)
        self._counters = HardLocation.create_counters()

    def write(self, word):
        for i, dim in enumerate(word._dims):
            self._counters[i][dim] += 1

    @staticmethod
    def create_counters():
        _counters = list()
        for i in xrange(ndim):
            _counters.append(np.array([0]*_axis_length))
        return _counters

    def __iter__(self):
        self.start = 0

    def __add__(self, other):
        result = HardLocation()
        for i, counter_dim in enumerate(result._counters):
            result._counters[i] = other._counters[i]+self._counters[i]

        result._dims = self._dims+other._dims
        return result

    @property
    def counters(self):
        return self._counters


class IntegerSDM(object):
    def __init__(self, n_hard_locations):
        self.hard_locations = [HardLocation() for i in xrange(n_hard_locations)]
        if _axis_length % 2 != 0:
            raise Exception('r should be even')
        # phi_inv = scipy.stats.norm.ppf(0.001) the calculation used below
        phi_inv = -3.0902323061678132 # for p=0.0001 phi_inv(p) = -3.0902...
        r_ = _axis_length
        self.access_sphere_radius = ((ndim*(r_**2+8)/48)**.5)*phi_inv+((ndim*r_)/4)

    def read(self, address, prev_distances=[8000]):  # TODO fix default []
        if len(prev_distances) > 100 or prev_distances[-1] < 100:
            return address

        hard_locations_in_radius = [location for location in self.hard_locations
                                    if address.distance(location) < self.access_sphere_radius]

        # adding the counters of locations in radius
        empty_location = HardLocation()
        total = empty_location
        for location in hard_locations_in_radius:
            total = total+location

        # creating the word from max of counters
        word = list()
        for counter in total.counters:
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

import time
start = time.time()
pam = IntegerSDM(1000)
import pickle
pickle.dump(pam, open('pam1000', 'wb'), -1)
print(time.time()-start)