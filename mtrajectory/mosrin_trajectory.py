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
        self.segments_length = 0.0
        self.stride_length = _stride_length
        self.segment_vector = obosthan.OVector2D(_stride_length, 0.0)
        self.segment_vector_angle = 0.0
        self.horizontal_BB = 0.0 # Horizontal dimension of the bounding box
        self.vertical_BB = 0.0 # Vertical dimension of the bounding box

    def calculate_BB(self, _start, _end):

        hmin, vmin = self.points[_start]

        hmax = hmin

        vmax = vmin

        for p in self.points[_start:_end]:
            if hmin < p[0]:
                hmin = p[0]
            if hmax > p[0]:
                hmax = p[0]
            if vmin < p[1]:
                vmin = p[1]
            if vmax > p[1]:
                vmax = p[1]

        return (abs(hmax-hmin),abs(vmax-vmin))

    def add_zero_curvature_segments(self, _numbers):

        i_point_index = self.points_num

        for i in range(_numbers):
            scale_amount = abs(float(self.stride_length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += self.stride_length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)

        return self.calculate_BB(i_point_index - 1, self.points_num)

    def add_constant_curvature_segments(self, _radius, _numbers):

        i_point_index = self.points_num

        for i in range(_numbers):
            d = mfind_deflection(_radius, self.stride_length)[0]
            self.segment_vector_angle += d
            scale_amount = abs(float(self.stride_length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += self.stride_length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)

        return self.calculate_BB(i_point_index - 1, self.points_num)

    def add_linear_curvature_segments(self, _start_radius, _end_radius, _numbers):

        a = mfind_clothoid_deflection_acceleration(_start_radius, _end_radius, self.stride_length*(_numbers), self.stride_length)
        di = mfind_deflection(_start_radius, self.stride_length)[0]

        i_point_index = self.points_num

        for i in range(_numbers):
            d = (i * a) + di
            self.segment_vector_angle += d
            scale_amount = abs(float(self.stride_length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += self.stride_length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)

        return self.calculate_BB(i_point_index-1, self.points_num)
