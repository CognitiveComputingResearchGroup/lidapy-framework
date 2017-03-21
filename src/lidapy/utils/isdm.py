from math import pi, sin, cos, atan
import numpy as np

TWO_PI = 2*pi

class ModularDimension(object):
    global TWO_PI

    def __init__(self, value, r):
        self._axis_length = r[1]-r[0]+1
        self.del_theta = TWO_PI/self._axis_length
        self.mag = 1
        self.value = value
        self._max = r[1]
        self._min = r[0]
        self._theta = self.value * self.del_theta

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, theta):
        self.value = int(theta/self.del_theta)
        self._theta = theta

    def __add__(self, other):
        result = ModularDimension(0, [0, 15])
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
        return ModularDimension(min([(self.value-other.value)%self._axis_length,
                    (other.value-self.value)%self._axis_length]), (self._min, self._max))

    def __mul__(self, other):
        return ModularDimension((self.value+other.value) % self._axis_length,
                                (self._min, self._max))

    def __invert__(self):
        return ModularDimension((self._max+1-self.value) % self._axis_length,
                                (self._min, self._max))


class MCRVector(object):

    def __init__(self, dims, r=(0, 15)):
        self._size = len(dims)
        if len(r) > 2 and r[0] > r[1]:
            raise ValueError
        self._max = r[1]
        self._min = r[0]
        self._axis_length = self._max-self._min+1
        self._dims = [ModularDimension(i, r) for i in dims]

    def __len__(self):
        return self._size

    def __invert__(self):
        dims_copy = self._dims.copy()
        np.apply_along_axis(lambda dim: ~dim, 0, dims_copy)
        return MCRVector(dims_copy, self.size, (self._min, self._max))

    def __mul__(self, other):
        if self.size != other.size:
            raise ValueError
        return MCRVector([(self[i]*other[i]).value for i in range(len(other))])

    def __add__(self, other):
        if self.size != other.size:
            raise ValueError
        return MCRVector([(self[i]+other[i]).value for i in range(len(other))])

    def distance(self, other):
        if self.size != other.size:
            raise ValueError
        return sum([(self[i]-other[i]).value for i in range(len(other))])

    def __getitem__(self, item):
        return self._dims[item]

    @property
    def size(self):
        return self._size

    @classmethod
    def randomvector(cls, size=2048, r=(0, 15)):
        _min = r[0]
        _max = r[1]
        _dims = np.random.random_integers(_min, _max, size)
        return cls(_dims, r)


class HardLocation(MCRVector):
    def __init__(self, size=2048, r=(0,15)):
        _vector = MCRVector.randomvector(size, r)._dims
        super(HardLocation, self).__init__(_vector, size, r)
        self._counter = HardLocation.create_counter(size,r)

    def write(self, word):
        for i, dim in enumerate(word):
            self._counters[i][dim] += 1

    def create_counter(n_dim=2048, r=(0,15)):
        axis_length = r[1]-r[0]+1
        _counters = list()
        for i in n_dim:
            _counters.append(np.array([0]*axis_length))
        return _counters

    def __add__(self, other):
        result = HardLocation(self.size, (self._min, self._max))
        for i, counter_dim in enumerate(result._counter):
           result._counter[i] = other._counter[i] + self._counter[i]

        result._dims = self._vector+other._vector
        return result


    @property
    def counters(self):
        return self._counters


class IntegerSDM(object):
    def __init__(self, n_hard_locations, n_dims, r):
        self._axis_length = r[1]-r[0]+1
        self._n_dims = n_dims
        self._r = r
        self.hard_locations = [HardLocation(n_dims, r) for i in n_hard_locations]
        # phi_inv = scipy.stats.norm.ppf(0.001) the calculation used below
        if self._axis_length%2 !=0:
            raise Exception('r should be even')
        phi_inv = -3.0902323061678132
        r = self._axis_length
        self.access_sphere_radius =((n_dims*(r**2+8)/48)**.5)*phi_inv+((n_dims*r)/4)

    def read(self, address, prev_distances=[]): #TODO fix default []
        if len(prev_distances) > 100 or prev_distances[-1] < 100:
            return address

        hard_locations_in_radius = [location for location in self.hard_locations
                                    if address.distance(location)<self.access_sphere_radius]
        empty_location = HardLocation(self._n_dims, self._r)
        total = empty_location
        for location in hard_locations_in_radius:
            total = total+location

        word = list()
        for counter in total.counters:
            word.append(np.argmax(counter))

        prev_distances.append(address.distance(word))
        return self.read(MCRVector(word, self._r), prev_distances)

    def write(self, word):
        hard_locations_in_radius = [location for location in self.hard_locations
                                        if word.distance(location)<self.access_sphere_radius]
        for location in hard_locations_in_radius:
            location.write(word)

