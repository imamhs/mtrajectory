# Copyright (c) 2020-2021, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

import obosthan
from .curve_dynamics import mfind_deflection, mfind_clothoid_deflection_acceleration

"""
Smooth trajectory object
"""

class Mtrajectory:
    def __init__(self, _x, _y, _stride_length):

        self.points = [(_x, _y)]
        self.points_num = 1
        self.stride_length = _stride_length
        self.segment_vector = obosthan.OVector2D(_stride_length, 0.0)
        self.segment_vector_angle = 0.0
        self.horizontal_space = 0.0
        self.vertical_space = 0.0

    def update_space(self):
        self.horizontal_space = abs(self.points[-1][0] - self.points[0][0])
        self.vertical_space = abs(self.points[-1][1] - self.points[0][1])

    def add_zero_curvature_segments(self, _numbers):

        for i in range(_numbers):
                scale_amount = abs(float(self.stride_length / self.segment_vector.length))
                self.segment_vector.scale(scale_amount)
                self.segment_vector.rotate(self.segment_vector_angle)
                self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
                self.points_num += 1

        self.update_space()

    def add_constant_curvature_segments(self, _radius, _numbers):

        for i in range(_numbers):
                d = mfind_deflection(_radius, self.stride_length)[0]
                self.segment_vector_angle += d
                scale_amount = abs(float(self.stride_length / self.segment_vector.length))
                self.segment_vector.scale(scale_amount)
                self.segment_vector.rotate(self.segment_vector_angle)
                self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
                self.points_num += 1

        self.update_space()

    def add_linear_curvature_segments(self, _start_radius, _end_radius, _numbers):

        a = mfind_clothoid_deflection_acceleration(_start_radius, _end_radius, self.stride_length*(_numbers), self.stride_length)
        di = mfind_deflection(_start_radius, self.stride_length)[0]

        for i in range(_numbers):
                d = (i * a) + di
                self.segment_vector_angle += d
                scale_amount = abs(float(self.stride_length / self.segment_vector.length))
                self.segment_vector.scale(scale_amount)
                self.segment_vector.rotate(self.segment_vector_angle)
                self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
                self.points_num += 1

        self.update_space()
