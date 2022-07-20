# Copyright (c) 2020-2022, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

import mtrajectory
from math import isclose

# Unit tests

assert isclose(mtrajectory.mfind_stride_length(23.0739, 50), 20.0, abs_tol=0.01) == True, "mfind_stride_length test failed"
assert isclose(mtrajectory.mfind_stride_length1(5, 50, 5), 3.7237, abs_tol=0.01) == True, "mfind_stride_length1 test failed"
assert isclose(mtrajectory.mfind_radius(23.0739, 20), 50.0, abs_tol=0.01) == True, "mfind_radius test failed"
assert isclose(mtrajectory.mfind_radius1(5.0, 5, 3.7237), 50.0, abs_tol=0.01) == True, "mfind_radius1 test failed"
assert isclose(mtrajectory.mfind_radius2(23.0739, 20), 49.6628, abs_tol=0.01) == True, "mfind_radius2 test failed"
assert isclose(mtrajectory.mfind_deflection(50, 20)[0], 23.0739, abs_tol=0.01) == True, "mfind_deflection test failed"
assert isclose(mtrajectory.mfind_deflection1(50, 5, 3.7237)[0], 5.0, abs_tol=0.01) == True, "mfind_deflection1 test failed"
