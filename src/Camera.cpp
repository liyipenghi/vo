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

#include <opencv2/opencv.hpp>
#include <Camera.h>

namespace VO {
    PinholeCamera::PinholeCamera(int width_, int height_, double fx_, double fy_, double ux_, double uy_, 
                    double d0, double d1, double d2, double d3, double d4) : 
                    BaseCamera(width_, height_),
                    fx(fx_), fy(fy_), ux(ux_), uy(uy_),
                    distorted(fabs(d0) > EPS),
                    undist_map1(height_, width_, CV_16SC2),
                    undist_map2(height_, width_, CV_16SC2)
    {
        d[0] = d0;
        d[1] = d1;
        d[2] = d2;
        d[3] = d3;
        d[4] = d4;
        
        cvK = (cv::Mat_<float>(3,3) << fx, 0.0, ux, 0.0, fy, uy, 0.0, 0.0, 1.0);
        cvD = (cv::Mat_<float>(1,5) << d[0], d[1], d[2], d[3], d[4]);
        cv::initUndistortRectifyMap(cvK, cvD, cv::Mat_<double>::eye(3,3),
                                    cvK, cv::Size(width,height), CV_16SC2, undist_map1,undist_map2);
        
        K << fx, 0.0, ux, 0.0, fy, uy, 0.0, 0.0, 1.0;
        invK = K.inverse(); 
    }
    PinholeCamera::~PinholeCamera() {}
    
    Vector3d PinholeCamera::world2camera(const Vector3d& p_w, const SE3& T_c_w) const
    {
	return T_c_w*p_w;
    }
    Vector3d PinholeCamera::camera2world(const Vector3d& p_c, const SE3& T_c_w) const
    {
	return T_c_w.inverse()*p_c;
    }
    Vector2d PinholeCamera::camera2pixel(const Vector3d& p_c) const 
    {
	return Vector2d(
	  fx*p_c(0,0)/p_c(2,0) + ux,
	  fy*p_c(1,0)/p_c(2,0) + uy
	);
    }
    Vector3d PinholeCamera::pixel2camera(const Vector2d& p_p, double depth) const
    {
	return Vector3d(
	  (p_p(0,0)-ux)*depth/fx,
	  (p_p(1,0)-uy)*depth/fy,
	  depth
	);
    }
    Vector2d PinholeCamera::world2pixel(const Vector3d& p_w, const SE3& T_c_w) const
    {
	return camera2pixel(world2camera(p_w, T_c_w));
    }
    Vector3d PinholeCamera::pixel2world(const Vector2d& p_p, const SE3& T_c_w, double depth) const
    {
	return camera2world(pixel2camera(p_p, depth), T_c_w);
    }



} // namespace VO