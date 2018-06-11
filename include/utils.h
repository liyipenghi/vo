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

#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace image_utils{
    // half pyramid down 
    void halfPyr(const cv::Mat& src_, cv::Mat& dst_);
    // create image pyramid to given level
    void createImgPyr(const cv::Mat& src_, std::vector<cv::Mat>& dst_, int level_);
    // cal shiTomasi score
    float shiTomasiScore(const cv::Mat& src_, int u_, int v_);

} // end of namespace image_utils

#endif