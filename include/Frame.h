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

#ifndef FRAME_H
#define FRAME_H


#include <sophus/se3.h>
#include <Camera.h>
#include <boost/noncopyable.hpp>
#include <common.h>

namespace VO {
    class Point_3d;
    struct Feature;

    typedef list<Feature*> Features;
    typedef vector<cv::Mat> ImgPyr;

    class Frame : boost::noncopyable {
    public:
        // methods
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef boost::shared_ptr<Frame> Ptr;  
	
	Frame();
        Frame(PinholeCamera* camera_, const cv::Mat& img_, double timestamp_);
        ~Frame();

        void initFrame(const cv::Mat& img_);

        void setKeyFrame();

        void addFeature(Feature* feature_);
	
	bool isInFrame(const Vector3d& pos_);
	
	inline Vector3d pos() {return Tw2c.inverse().translation();}

        inline static void jacobian_xyz2uv(const Vector3d& xyz_in_f_, Matrix<double, 2, 6>& jac_)
        {
            const double x = xyz_in_f_[0];
            const double y = xyz_in_f_[1];
            const double z_inv = 1.0/xyz_in_f_[2];
            const double z_inv2 = z_inv*z_inv;

            jac_(0,0) = -z_inv;
            jac_(0,1) = 0.0;
            jac_(0,2) = x*z_inv2;
            jac_(0,3) = y*jac_(0,2);
            jac_(0,4) = -(1.0 + x*jac_(0,2));
            jac_(0,5) = y*z_inv;

            jac_(1,0) = 0.0;
            jac_(1,1) = -z_inv;
            jac_(1,2) = y*z_inv2;
            jac_(1,3) = 1.0+y*jac_(1,2);
            jac_(1,4) = -jac_(0,3);
            jac_(1,5) = -x*z_inv;
        }

    public:
        // members
        static int frame_counter;
        int id;
        double timestamp;
        PinholeCamera* camera;
        Sophus::SE3 Tw2c; // transformation from world to camera
        Matrix<double, 6, 6> Cov; // covariance of Tw2c
        ImgPyr img_pyr;
        Features features;
        bool is_keyframe;
    };
} // namespace VO

#endif
