#!/usr/bin/env python2

from math import cos, sin, fmod, pi, atan2, isinf, isnan
from collections import Sequence


def isfinite(value):
    return not isnan(value) and not isinf(value)


def rad_bounded(value):
    value = fmod(value + pi, 2 * pi)
    if 0 > value:
        value += 2 * pi
    return value - pi


class PolarHistogram(Sequence):
    class Vector:
        def __init__(self, angle, range=0.0):
            self._angle = angle
            self._range = range

        def get_range(self):
            return self._range

        def get_angle(self):
            return self._angle

        def get_point(self):
            return [self._range * cos(self._angle), self._range * sin(self._angle)]

        def set_range(self, range):
            self._range = range

        def isfinite(self):
            return isfinite(self._range)

    def __init__(self, num_buckets, init_range):
        self._bucket_size = 2 * pi / num_buckets
        self._half_bucket_size = self._bucket_size / 2

        self._histogram = []
        angle = 0
        while 2 * pi > angle:
            self._histogram.append(PolarHistogram.Vector(angle, init_range))
            angle += self._bucket_size

    def rad_bounded(self, value):
        value = fmod(value + pi, 2 * pi)
        if 0 > value:
            value += 2 * pi
        return value - pi

    def pos_rad(self, value):
        value = self.rad_bounded(value)
        if 0 > value:
            return value + (2 * pi)
        return value

    def bucket_size(self):
        return self._bucket_size

    def num_buckets(self):
        return len(self._histogram)

    def get(self, direction):
        direction += self._half_bucket_size
        return self._histogram[int(self.pos_rad(direction) / self._bucket_size)]

    def get_range(self, direction):
        direction += self._half_bucket_size
        return self._histogram[int(self.pos_rad(direction) / self._bucket_size)].get_range()

    def set_range(self, direction, range):
        direction += self._half_bucket_size
        self._histogram[int(self.pos_rad(
            direction) / self._bucket_size)].set_range(range)

    def get_point(self, direction):
        direction += self._half_bucket_size
        return self._histogram[int(self.pos_rad(direction) / self._bucket_size)].get_point()

    def insert_point(self, point):
        self.set_range(atan2(point[1], point[0]), np.linalg.norm(point))

    def isfinite(self, direction):
        return self.get(direction).isfinite()

    def __getitem__(self, i):
        return self._histogram[i]

    def __len__(self):
        return len(self._histogram)
