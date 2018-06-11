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

#ifndef CAMERA_H
#define CAMERA_H

#include <common.h>

namespace VO {
    class BaseCamera {
    public:
        BaseCamera() {};
        BaseCamera(int width_, int height_) : 
            width(width_), height(height_)
        {};

        virtual ~BaseCamera() {};

        inline int width_() const {return width;}
        inline int height_() const {return height;}
        
        //
        virtual Vector3d world2camera(const Vector3d& p_w, const SE3& T_c_w) const = 0;
	virtual Vector3d camera2world(const Vector3d& p_c, const SE3& T_c_w) const = 0;
	virtual Vector2d camera2pixel(const Vector3d& p_c) const = 0;
	virtual Vector3d pixel2camera(const Vector2d& p_p, double depth=1) const = 0;
	virtual Vector3d pixel2world(const Vector2d& p_p, const SE3& T_c_w,double depth=1) const = 0;
	virtual Vector2d world2pixel(const Vector3d& p_w, const SE3& T_c_w) const = 0;
    protected:
        int width;
        int height;
    }; // BaseCamera

    class PinholeCamera : public BaseCamera {
    public:
        // methid
        PinholeCamera(int width_, int height_, double fx_, double fy_, double ux_, double uy_, 
                    double d0 = 0.0, double d1 = 0.0, double d2 = 0.0, double d3 = 0.0, double d4 = 0.0);
        ~PinholeCamera();
	
	virtual Vector3d world2camera(const Vector3d& p_w, const SE3& T_c_w) const;
	virtual Vector3d camera2world(const Vector3d& p_c, const SE3& T_c_w) const;
	virtual Vector2d camera2pixel(const Vector3d& p_c) const;
	virtual Vector3d pixel2camera(const Vector2d& p_p, double depth=1) const;
	virtual Vector3d pixel2world(const Vector2d& p_p, const SE3& T_c_w,double depth=1) const;
	virtual Vector2d world2pixel(const Vector3d& p_w, const SE3& T_c_w) const;
    public:
        // member
        const double fx;
        const double fy;
        const double ux;
        const double uy;
        double d[5]; 
        bool distorted;
        cv::Mat cvK;    // intrinsic matrix in opencv
        cv::Mat cvD;    // distortion parameter vector in opencv 
        cv::Mat undist_map1;
        cv::Mat undist_map2;
        Matrix3d K;
        Matrix3d invK;
    }; // PinholeCamera
} // namespace VO

#endif