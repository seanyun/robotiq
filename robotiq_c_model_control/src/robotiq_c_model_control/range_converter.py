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

class RangeConverter:
    def __init__(self, r1_min, r1_max, r2_min, r2_max):
        self.r1_min_ = r1_min
        self.r1_max_ = r1_max
        self.r2_min_ = r2_min
        self.r2_max_ = r2_max
        self.r1_range_ = self.r1_max_ - self.r1_min_
        self.r2_range_ = self.r2_max_ - self.r2_min_

    def r1_to_r2(self, r1_val):
        ratio = (r1_val - self.r1_min_) / float(self.r1_range_)
        return ratio * self.r2_range_ + self.r2_min_

    def r2_to_r1(self, r2_val):
        ratio = (r2_val - self.r2_min_) / float(self.r2_range_)
        return ratio * self.r1_range_ + self.r1_min_
