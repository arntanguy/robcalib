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
#include <chrono>


template<typename TimeT = std::chrono::milliseconds>
struct measure
{
template<typename F, typename ...Args>
static TimeT duration(F&& func, Args&&... args)
{
    auto start = std::chrono::high_resolution_clock::now();
    std::forward<decltype(func)>(func)(std::forward<Args>(args)...);
    return std::chrono::duration_cast<TimeT>(std::chrono::high_resolution_clock::now()-start);
}
};

