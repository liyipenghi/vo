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

#include <Point3d.h>
#include <Frame.h>
#include <Feature.h>

namespace VO {
   
    int Point_3d::point_counter = 0;

    Point_3d::Point_3d(const Vector3d& pos_) :
    id(point_counter++),
    pos(pos_),
    n_obs(0),
    match_times(1),
    visible_times(1),
    state(STATE_GOOD)
    {}

    Point_3d::Point_3d(const Vector3d& pos_, Feature* feature_) :
        id(point_counter++),
        pos(pos_),
        n_obs(1),
        match_times(1),
	visible_times(1),
	state(STATE_GOOD)
    {
        observation.push_front(feature_);
    }

    Point_3d::~Point_3d()
    {}

    void Point_3d::addObservation(Feature* feature_)
    {
        observation.push_front(feature_);
        ++n_obs;
    }

    bool Point_3d::delObservation(Frame* frame_)
    {
        for (auto it=observation.begin(); it!=observation.end();++it)
        {
            if ((*it)->frame == frame_)
            {
                observation.erase(it);
                return true;
            }
        }

        return false;
    }
    
    Point_3d::Ptr Point_3d::createPoint3d()
    {
	return nullptr;
    }
    Point_3d::Ptr Point_3d::createPoint3d(const Vector3d& pos_, const Vector3d& pos_norm_, Feature* feature_)
    {
	return nullptr;
    }


} // namespace VO
