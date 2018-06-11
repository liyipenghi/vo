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

#ifndef POINT_3D_H
#define POINT_3D_H

#include <boost/noncopyable.hpp>
#include <common.h>

namespace VO {
    struct Feature;
    class Point_3d : boost::noncopyable {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        typedef boost::shared_ptr<Point_3d> Ptr;

        enum PointState {
            STATE_DELETED,
            STTAE_CANDIDATE,
            STATE_UNKNOWN,
            STATE_GOOD
        };

        Point_3d(const Vector3d& pos_);
        Point_3d(const Vector3d& pos_, Feature* feature_);
        ~Point_3d();
	
	static Point_3d::Ptr createPoint3d();
	static Point_3d::Ptr createPoint3d(const Vector3d& pos_, 
					   const Vector3d& pos_norm_,
					   Feature* feature_
					  );

        void addObservation(Feature* feature_); // add a new observation
        bool delObservation(Frame* frame_);     // delete obs in frame
	inline cv::Point3f getPoint() const {
	    return cv::Point3f(pos(0,0), pos(1,0), pos(2,0));
	}
    public:
        PointState state;
        static int point_counter;
        int id;
        Vector3d pos;
        Vector3d pos_norm;  // surface normal
        Matrix3d norm_info; // inverse covariance matrix of normal estimation
	cv::Mat desc;
	
        int match_times;
	int visible_times;
        
        std::list<Feature*> observation; // corresponding
        size_t n_obs;

    };
} // namespace VO

#endif
