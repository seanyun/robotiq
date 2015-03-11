#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, SRI, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2014, SRI, Inc.

from range_converter import RangeConverter

def clamp_to_byte(value):
    if value < 0:
        return 0
    if value > 255:
        return 255
    return int(value)

class CModel85Conversions:
    def __init__(self):
        # These values were taken from the Robotiq C-model 85 instruction manual at
        # http://robotiq.com/media/ROBOTIQ-INSTRUCTIONMANUAL-C-MODEL-ROBOT-GRIPPER.pdf

        # Distance is finger separation in meters, which is in the
        # opposite direction from the command values.
        self.dist_ = RangeConverter(0.0, 87.0, 230.0, 13.0)

        # Speed is in meters/second
        self.speed_ = RangeConverter(0.013, 0.100, 0.0, 255.0)

        # Force is in Newtons
        self.force_ = RangeConverter(30.0, 100.0, 0.0, 255.0)

    def dist_to_command(self, distance):
        return clamp_to_byte(self.dist_.r1_to_r2(distance))

    def command_to_dist(self, command):
        return self.dist_.r2_to_r1(command)
        
    def speed_to_command(self, speed):
        return clamp_to_byte(self.speed_.r1_to_r2(speed))

    def command_to_speed(self, command):
        return self.speed_.r2_to_r1(command)
        
    def force_to_command(self, force):
        return clamp_to_byte(self.force_.r1_to_r2(force))

    def command_to_force(self, command):
        return self.force_.r2_to_r1(command)
