from math import pi, sin, cos, atan
import shutil
from functools import partial
'''
try:
    import msgpack as pickle
    print("msgpack imported")
except ImportError:
'''
import pickle
import os
import tensorflow as tf
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
memory_location = 'pam'
same_vector_distance_threshold = 100

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

    return round(result/del_theta) % _axis_length, magnitude

# Check add operation
# for a, b in itertools.combinations(range(16), 2):
#    print(a, b, add_two_dimensions(a, 1, b, 1)[0])


class MCRVector(object):

    __slots__ = ['_dims', '_factor', '_magnitudes_internal']

    def __init__(self, dims, factor=1, _mag=np.array([1]*ndim)):
        if len(dims) != ndim:
            raise ValueError
        self._dims = dims
        self._factor = factor
        self._magnitudes_internal = _mag

    @property
    def _magnitudes(self):
        magnitudes = self._magnitudes_internal
        self._magnitudes_internal = [1] * ndim
        return magnitudes

    @classmethod
    def random_vector(cls, factor=1, _mag=np.array([1]*ndim)):
        _dims = np.random.random_integers(r_min, r_max, ndim)
        return cls(_dims, factor=factor, _mag=_mag)

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
        # Optimization to prevent a lot of multiplications because _factor is mostly 1
        self_magnitudes = self._magnitudes*self.factor if self.factor != 1 else self._magnitudes
        other_magnitudes = other._magnitudes*other.factor if other.factor != 1 else other._magnitudes

        addition_result = [add_two_dimensions(*param_values) for param_values in zip(self.dims,
                                                                                     self_magnitudes,
                                                                                     other.dims,
                                                                                     other_magnitudes)]

        return MCRVector(np.array([i[0] for i in addition_result]), _mag=[i[1] for i in addition_result])

    def __getitem__(self, item):
        return self._dims[item]

    def distance(self, other):
        return MCRVector.distance_between(self, other)

    @staticmethod
    def distance_between(x, y):
        return sum([min((self_dim-other_dim) % _axis_length,
                        (other_dim-self_dim) % _axis_length) for self_dim, other_dim in zip(x.dims, y.dims)])

    @property
    def dims(self):
        return self._dims

    @property
    def factor(self):
        return self._factor

    def get_noisy_copy(self, pct=.1):
        chosen_dims = np.random.choice(range(ndim), size=int(ndim*pct))
        new_dims = self.dims.copy()
        for i in chosen_dims:
            new_dims[i] = np.random.random_integers(r_min, r_max)
        return MCRVector(new_dims, factor=self._factor)


class HardLocation(MCRVector):

    __slots__ = ['_name', '_dims', '_factor', '_magnitudes_internal']

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

    def __hash__(self):
        return hash(self.name)

class IntegerSDM(object):

    @staticmethod
    @lru_cache(maxsize=100)
    def retrieve_counters(hard_location_name):
        try:
            mem_file = open(os.path.join(memory_location, hard_location_name), 'rb')
        except FileNotFoundError:
            return None
        return pickle.load(mem_file)

    @staticmethod
    def store_counters(hard_location_name, counters):
        mem_file = open(os.path.join(memory_location, hard_location_name), 'wb')
        pickle.dump(counters, mem_file)


class CachedIntegerSDM(IntegerSDM):
    def __init__(self, n_hard_locations):
        self.hard_locations = [HardLocation(name='location'+str(i)) for i in range(n_hard_locations)]
        if _axis_length % 2 != 0:
            raise Exception('r should be even')
        # phi_inv = scipy.stats.norm.ppf(0.001) the calculation used below
        phi_inv = -3.0902323061678132 # for p=0.001 phi_inv(p) = -3.0902...
        r_ = _axis_length
        self.access_sphere_radius = ((ndim*(r_**2+8)/48)**.5)*phi_inv+((ndim*r_)/4)
        try:
            os.mkdir(memory_location)
            created = True
        except FileExistsError:
            print("This location", os.path.join(os.getcwd(), memory_location)," already stores a memory. Run from a different location")
            created = False
        self._created = created

    def hard_locations_in_radius(self, vector, radius=-1):
        if radius == -1:
            radius = self.access_sphere_radius

        hard_locations_in_radius = [location for location in self.hard_locations
                                    if vector.distance(location) < radius]

        return hard_locations_in_radius

    def read(self, address, prev_distances=[]):  # TODO fix default []
        if len(prev_distances) > 2:
            if prev_distances[-1] < same_vector_distance_threshold or len(prev_distances) > 100:
                print(prev_distances)
                print("read by convergence")
                return address

            if np.mean([prev_distances[i]-prev_distances[i-1] for i in range(1, len(prev_distances))]) > 0:
                raise Exception("cannot be read")

        locations_in_radius = self.hard_locations_in_radius(address)

        if len(locations_in_radius) < 0:
            raise Exception("cannot be read")
        # adding the counters of locations in radius
        total = HardLocation.get_empty_counter()
        for location in locations_in_radius:
            total = location.add_counters(total)

        # creating the word from max of counters
        word = list()
        for counter in total:
            word.append(np.argmax(counter))
        word = MCRVector(np.array(word))

        prev_distances.append(address.distance(word))

        return self.read(word, prev_distances)

    def write(self, word):
        locations_in_radius = self.hard_locations_in_radius(word)

        if len(locations_in_radius) < 1:
            raise Exception("cannot be written")
        for location in locations_in_radius:
            location.write(word)

    def __del__(self):
        if self._created:
            shutil.rmtree(memory_location)

class NPIntegerSDM(CachedIntegerSDM):
    def __init__(self, n_hard_locations):
        super(NPIntegerSDM, self).__init__(n_hard_locations)
        hard_locations = np.array([location.dims for location in self.hard_locations])
        self._hard_locations_tensor = hard_locations

    def hard_locations_in_radius(self, vector, radius=-1):
        if not isinstance(vector, MCRVector):
            raise ValueError('Input vector must be an object of MCRVector')
        if radius == -1:
            radius = self.access_sphere_radius
        mod1_diff = (self._hard_locations_tensor-vector.dims) % _axis_length
        mod2_diff = (vector.dims-self._hard_locations_tensor) % _axis_length
        min_diff = np.minimum(mod1_diff, mod2_diff)
        distances = np.sum(min_diff, axis=1)
        in_radius = distances < radius
        locations = np.where(in_radius)
        return [self.hard_locations[location] for location in locations[0]]


class TFIntegerSDM(CachedIntegerSDM):
    def __init__(self, n_hard_locations):
        super(TFIntegerSDM, self).__init__(n_hard_locations)
        hard_locations = np.array([location.dims for location in self.hard_locations])
        graph = tf.Graph()
        with graph.as_default():
            self._hard_locations_tensor = tf.constant(hard_locations)
            self._query_vector = tf.placeholder(dtype=tf.int32, shape=(ndim))
            self._query_radius = tf.placeholder(dtype=tf.int32)
            mod1_diff = (self._hard_locations_tensor-self._query_vector) % _axis_length
            mod2_diff = (self._query_vector-self._hard_locations_tensor) % _axis_length
            min_diff = tf.minimum(mod1_diff, mod2_diff)
            distances = tf.reduce_sum(min_diff, axis=1)
            in_radius = distances < self._query_radius
            self._locations = tf.where(in_radius)
        self._distance_checker = tf.Session(graph=graph)

    def hard_locations_in_radius(self, vector, radius=-1):
        if not isinstance(vector, MCRVector):
            raise ValueError('Input vector must be an object of MCRVector')
        if radius == -1:
            radius = self.access_sphere_radius

        locations = self._distance_checker.run(self._locations, feed_dict={self._query_vector: vector.dims,
                                                                           self._query_radius: radius})
        return [self.hard_locations[location_index[0]] for location_index in locations]


if __name__ == '__main__':

    pam = NPIntegerSDM(100000)

    cake = MCRVector.random_vector()
    ram = MCRVector.random_vector()
    apple = MCRVector.random_vector()
    sita = MCRVector.random_vector()
    eat = ram*cake + sita*apple

    cake.distance(ram)
    print(eat.distance(ram))

    pam.write(cake)
    pam.write(ram)
    pam.write(apple)
    pam.write(sita)

    noisy_ram = ram.get_noisy_copy()
    read_ram = pam.read(noisy_ram)
    noisy_ram.distance(ram)
    v1 = eat*(~ram)
    print(v1.distance(ram))
    print(v1.distance(eat))
    v1 = pam.read(v1)
    print(v1.distance(cake))
    print(v1.distance(sita))
    print(cake.distance(sita))
