// Copyright (C) 2018 Yipeng Li
// 
// This file is part of vo.
// 
// vo is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// vo is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with vo.  If not, see <http://www.gnu.org/licenses/>.
// 

#ifndef MATH_FUNS_H
#define MATH_FUNS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/StdVector>
#include <sophus/se3.h>

namespace Vision {
  
  inline double norm_max(const Eigen::VectorXd& v)
  {
    double max = -1;
    for (int i=0; i<v.size(); i++)
    {
      double abs = fabs(v[i]);
      if (abs > max)
      {
	max = abs;
      }
    }
    return max;
  }

  
} // end of Vision
#endif