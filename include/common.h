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

#ifndef COMMON_H
#define COMMON_H

// std
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <math.h>

// eigen
#include <Eigen/Core>
// opencv
#include <opencv2/opencv.hpp>
// sophus
#include <sophus/se3.h>
// boost
#include <boost/shared_ptr.hpp>

namespace VO{
    #define EPS 0.00000001
    using namespace Eigen;
    using namespace Sophus;
    using namespace std;
    class Frame;
    typedef boost::shared_ptr<Frame> FramePtr;
} // namespace VO

#endif
