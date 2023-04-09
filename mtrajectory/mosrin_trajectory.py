# Copyright (c) 2020-2023, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

import obosthan
from .curve_dynamics import mfind_deflection, mfind_deflection1, mfind_clothoid_deflection_acceleration

"""
Trajectory generation object
"""

class Mtrajectory:
    def __init__(self, _x, _y, _step_length):

        self.points = [(_x, _y)]
        self.points_num = 1
        self.segments_length = 0.0
        self.step_length = _step_length
        self.pre_step_length = 0.0
        self.segment_vector = obosthan.OVector2D(_step_length, 0.0)
        self.segment_vector_angle = 0.0
        self.horizontal_BB = 0.0  # Horizontal dimension of the bounding box
        self.vertical_BB = 0.0  # Vertical dimension of the bounding box

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

        return (abs(hmax-hmin), abs(vmax-vmin))

    def add_zero_curvature_segments(self, _numbers):

        i_point_index = self.points_num

        for i in range(_numbers):
            scale_amount = abs(float(self.step_length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += self.step_length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)
        self.pre_step_length = self.step_length

        return self.calculate_BB(i_point_index - 1, self.points_num)

    def add_constant_curvature_segments(self, _radius, _numbers, _con=False):

        i_point_index = self.points_num

        dcon = mfind_deflection1(_radius, self.pre_step_length, self.step_length)[0]
        d = mfind_deflection(_radius, self.step_length)[0]

        for i in range(_numbers):
            if i == 0 and _con is True:
                self.segment_vector_angle += dcon
            else:
                self.segment_vector_angle += d
            scale_amount = abs(float(self.step_length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += self.step_length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)
        self.pre_step_length = self.step_length

        return self.calculate_BB(i_point_index - 1, self.points_num)

    def add_linear_curvature_segments(self, _start_radius, _end_radius, _numbers, _con=False):

        a = mfind_clothoid_deflection_acceleration(_start_radius, _end_radius, self.step_length*(_numbers), self.step_length)
        dcon = mfind_deflection1(_start_radius, self.pre_step_length, self.step_length)[0]
        di = mfind_deflection(_start_radius, self.step_length)[0]

        i_point_index = self.points_num

        for i in range(_numbers):
            if i == 0 and _con is True:
                d = (i * a) + dcon
                self.segment_vector_angle += d
            else:
                d = (i * a) + di
                self.segment_vector_angle += d
            scale_amount = abs(float(self.step_length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += self.step_length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)
        self.pre_step_length = self.step_length

        return self.calculate_BB(i_point_index-1, self.points_num)

    def add_constant_acceleration_segments(self, _acceleration, _start_radius, _numbers, _con=False):

        di = mfind_deflection(_start_radius, self.step_length)[0]
        dcon = mfind_deflection1(_start_radius, self.pre_step_length, self.step_length)[0]

        i_point_index = self.points_num

        for i in range(_numbers):
            if i == 0 and _con is True:
                d = (i * _acceleration) + dcon
                self.segment_vector_angle += d
            else:
                d = (i * _acceleration) + di
                self.segment_vector_angle += d
            scale_amount = abs(float(self.step_length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0],
                                                 self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += self.step_length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)
        self.pre_step_length = self.step_length

        return self.calculate_BB(i_point_index - 1, self.points_num)

    def add_constant_deflection_segments(self, _deflection_c, _numbers, _deflection_i=None):

        i_point_index = self.points_num

        for i in range(_numbers):
            if i == 0 and (_deflection_i is not None):
                self.segment_vector_angle += _deflection_i
            else:
                self.segment_vector_angle += _deflection_c

            scale_amount = abs(float(self.step_length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += self.step_length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)
        self.pre_step_length = self.step_length

        return self.calculate_BB(i_point_index - 1, self.points_num)

    def add_segments_from_points(self, _plist):

        sz = len(_plist)

        if sz < 2:
            return False

        i_point_index = self.points_num

        for i in range(1, sz):
            pv = obosthan.OVector2D(0.0, 0.0)
            pv.define_line(_plist[i-1][0], _plist[i-1][1], _plist[i][0], _plist[i][1])

            if self.points_num > 1:
                lpv = obosthan.OVector2D(0.0, 0.0)
                lpv.define_line(self.points[-2][0], self.points[-2][1], self.points[-1][0], self.points[-1][1])
                self.segment_vector_angle += lpv.angle_to(pv)
            else:
                self.segment_vector_angle += pv.angle

            scale_amount = abs(float(pv.length / self.segment_vector.length))
            self.segment_vector.scale(scale_amount)
            self.segment_vector.rotate(self.segment_vector_angle)
            self.points.append(obosthan.OPoint2D(self.points[-1][0] + self.segment_vector[0], self.points[-1][1] + self.segment_vector[1]))
            self.points_num += 1
            self.segments_length += pv.length

        self.horizontal_BB, self.vertical_BB = self.calculate_BB(0, self.points_num)
        self.pre_step_length = pv.length

        return self.calculate_BB(i_point_index - 1, self.points_num)





