// Copyright 2017-2018 CNRS-UM LIRMM
// Copyright 2017-2018 Arnaud TANGUY <arnaud.tanguy@lirmm.fr>
//
// This file is part of robcalib.
//
// robcalib is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// robcalib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with robcalib.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <robcalib/calib_data.h>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <sophus/se3.hpp>
#include <thread>
#include <vector>

namespace robcalib
{
class InputModule
{
 public:
  using data_callback_t = std::function<void(const std::shared_ptr<CalibData>)>;

 protected:
  std::mutex data_mutex_;
  std::shared_ptr<CalibData> data_;
  data_callback_t data_callback_;
  int rate_ = 100;

 public:
  void set_data_callback(data_callback_t callback) {
    LOG(info) << "InputModule set_data_callback" << std::endl;
    data_callback_ = callback;
  }
  std::shared_ptr<CalibData> data()
  {
    std::lock_guard<std::mutex> l(data_mutex_);
    return data_;
  }
};

} /* robcalib */
