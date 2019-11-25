/*
     Title     : low_pass_filter.cpp
     Project   : moveit_jog_arm
     Created   : 1/11/2019
     Author    : Andy Zelenak

BSD 3-Clause License

Copyright (c) 2019, Los Alamos National Security, LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <moveit_jog_arm/low_pass_filter.h>

namespace moveit_jog_arm
{
LowPassFilter::LowPassFilter(double low_pass_filter_coeff)
{
  filter_coeff_ = low_pass_filter_coeff;
}

void LowPassFilter::reset(double data)
{
  previous_measurements_[0] = data;
  previous_measurements_[1] = data;

  previous_filtered_measurement_ = data;
}

double LowPassFilter::filter(double new_measurement)
{
  // Push in the new measurement
  previous_measurements_[1] = previous_measurements_[0];
  previous_measurements_[0] = new_measurement;

  double new_filtered_msrmt = (1. / (1. + filter_coeff_)) * (previous_measurements_[1] + previous_measurements_[0] -
                                                             (-filter_coeff_ + 1.) * previous_filtered_measurement_);

  // Store the new filtered measurement
  previous_filtered_measurement_ = new_filtered_msrmt;

  return new_filtered_msrmt;
}
}  // namespace moveit_jog_arm
