# BSD 3-Clause License
#
# Copyright 2026 Eurecat
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Eurecat nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Fallbacks for ROS/Python packages that may not be installed when running the tests."""

import importlib.util
import math
import sys
import types


def _is_available(module_name):
    try:
        return importlib.util.find_spec(module_name) is not None
    except ImportError:
        return False


def _quaternion_from_euler(roll, pitch, yaw):
    # Same convention as tf_transformations (static 'sxyz' axes); returns [x, y, z, w].
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class _NavigateToPoseGoal:

    def __init__(self):
        self.pose = None


class _NavigateToPose:
    Goal = _NavigateToPoseGoal


if not _is_available('tf_transformations'):
    _tf_transformations = types.ModuleType('tf_transformations')
    _tf_transformations.quaternion_from_euler = _quaternion_from_euler
    _tf_transformations.euler_from_quaternion = _euler_from_quaternion
    sys.modules['tf_transformations'] = _tf_transformations

if not _is_available('nav2_msgs.action'):
    _nav2_msgs = types.ModuleType('nav2_msgs')
    _nav2_msgs_action = types.ModuleType('nav2_msgs.action')
    _nav2_msgs_action.NavigateToPose = _NavigateToPose
    _nav2_msgs.action = _nav2_msgs_action
    sys.modules['nav2_msgs'] = _nav2_msgs
    sys.modules['nav2_msgs.action'] = _nav2_msgs_action
