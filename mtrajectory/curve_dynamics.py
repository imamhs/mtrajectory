# Copyright (c) 2020-2021, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

import math

"""
Curve dynamics routines
"""

STRAIGHT_RADIUS = 100000000000000000

def mfind_stride_length(deflection, turning_radius):

    angle = (deflection - 180) / 2
    num = 2 * math.cos(math.radians(angle))

    return turning_radius * num

def mfind_radius(deflection, stride_length):

    angle = (deflection - 180) / 2
    num = 2 * math.cos(math.radians(angle))

    return stride_length / num

def mfind_radius1(deflection, stride_length):

    # less accurate approach (derived from stride omega and speed)

    return stride_length / math.radians(deflection)

def mfind_deflection(turning_radius, stride_length):
    # this assumes strides length to change from one stride to next

    num = (stride_length**2)
    den = 2*stride_length*turning_radius
    stride_angle = math.degrees(math.acos(num/den))*2
    return ((180-stride_angle), stride_angle)

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
