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

#include <stdexcept>
#include <Frame.h>
#include <Feature.h>
#include <Point3d.h>
#include <iostream>
#include <vector>
#include <string>
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <utils.h>
#include <Param.h>

namespace VO {
    //using namespace std;
    //using namespace cv;

    int Frame::frame_counter = 0;
    
    Frame::Frame():
      id(-1), timestamp(-1), camera(nullptr), is_keyframe(false)
    {
	
    }

    
    Frame::Frame(PinholeCamera* camera_, const cv::Mat& img_, double timestamp_) :
        id(frame_counter++),
        timestamp(timestamp_),
        camera(camera_),
        is_keyframe(false)
    {
        initFrame(img_);
    }

    Frame::~Frame()
    {

    }

    void Frame::initFrame(const cv::Mat& img_)
    {
        if (img_.empty() || img_.type() != CV_8UC1 || img_.cols != camera->width_() || img_.rows != camera->height_())
            throw std::runtime_error("Image error!");

        std::for_each(features.begin(), features.end(), [&](Feature* ftr){ ftr = NULL; });

        image_utils::createImgPyr(img_, img_pyr, Param::get<int>("pyr_level"));
    }

    void Frame::setKeyFrame()
    {
        is_keyframe = true;

    }
    bool Frame::isInFrame(const Vector3d& pos_)
    {
	Vector3d p_c = camera->world2camera(pos_, Tw2c);
	if (p_c(2.0)<0) return false;
	Vector2d p_p = camera->world2pixel(pos_, Tw2c);
	return p_p(0,0)>0 && p_p(1,0)>0 && p_p(0,0)<camera->width_() && p_p(1,0) < camera->height_();
    }

} // namespace VO
