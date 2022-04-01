# Copyright (c) 2020-2022, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

import math

"""
Curve dynamics routines
"""

STRAIGHT_RADIUS = 100000000000000000


def mfind_stride_length(deflection, turning_radius):
    # this assumes strides length to not change from first stride to next one

    angle = (deflection - 180) / 2
    num = 2 * math.cos(math.radians(angle))

    return turning_radius * num


def mfind_stride_length1(deflection, turning_radius, stride_length_f):

    angle = math.radians(180 - deflection)

    c = 4 * turning_radius * (math.sin(angle)/2)

    stride_length_f_OA = math.asin((stride_length_f*math.sin(angle))/c)

    stride_length_n_OA = math.radians(deflection) - stride_length_f_OA

    return (stride_length_f * math.sin(stride_length_n_OA)) / math.sin(stride_length_f_OA)


def mfind_radius(deflection, stride_length):
    # this assumes strides length to not change from first stride to next one

    angle = (deflection - 180) / 2
    num = 2 * math.cos(math.radians(angle))

    return stride_length / num


def mfind_radius1(deflection, stride_length_f, stride_length_n):
    # this assumes strides length to change from first stride to next one

    angle = math.radians(180 - deflection)
    area = stride_length_f * stride_length_n * (math.sin(angle)/2)
    displacement =  math.sqrt((stride_length_f**2) + (stride_length_n**2) - (2*stride_length_f*stride_length_n*math.cos(angle)))

    return (stride_length_f*stride_length_n*displacement) / (4*area)


def mfind_radius2(deflection, stride_length):
    # less accurate approach (derived from stride omega and speed)
    # this assumes strides length to not change from first stride to next one

    return stride_length / math.radians(deflection)


def mfind_deflection(turning_radius, stride_length):
    # this assumes strides length to not change from first stride to next one

    num = stride_length
    den = 2*turning_radius
    angle = math.degrees(math.acos(num/den))*2
    return ((180-angle), angle)


def mfind_deflection1(turning_radius, stride_length_f, stride_length_n):
    # this assumes strides length to change from first stride to next one

    num = stride_length_f
    den = 2*turning_radius
    angle1 = math.degrees(math.acos(num/den))

    num = stride_length_n
    den = 2*turning_radius
    angle2 = math.degrees(math.acos(num/den))

    angle = angle1 + angle2

    return ((180-angle), angle)


def mfind_clothoid_deflection_acceleration(start_turning_radius, end_turning_radius, curve_length, stride_length):

    d1 = mfind_deflection(start_turning_radius, stride_length)[0]
    d2 = mfind_deflection(end_turning_radius, stride_length)[0]
    return (d2-d1)/(curve_length/stride_length)


def mfind_clothoid_heading(start_turning_radius, end_turning_radius, nsegments, stride_length):

    a = mfind_clothoid_deflection_acceleration(start_turning_radius, end_turning_radius, nsegments*stride_length, stride_length)
    di = mfind_deflection(start_turning_radius, stride_length)[0]
    angle = 0

    for i in range(nsegments):
        d = (i * a) + di
        angle += d

    return angle


def mfind_clothoid_radius(angular_acceleration, curve_length, stride_length):

    num_cos = math.cos((math.pi*angular_acceleration*((curve_length/stride_length)-1))/180)
    num = math.sqrt(2)*(stride_length**2)*math.sqrt((2*(stride_length**2)*num_cos)+(2*(stride_length**2)))
    den_cos = math.cos((math.pi*angular_acceleration*((curve_length/stride_length)-1))/90)
    den = 2 * math.sqrt(-1*(stride_length**4)*(den_cos-1))
    R = num/den
    return R


def mfind_jerk(instance_radius1, instance_radius2, instance_speed, instance_time):

    ca1 = (instance_speed**2)/instance_radius1
    ca2 = (instance_speed**2)/instance_radius2
    jerk = (ca2-ca1)/instance_time
    return jerk
