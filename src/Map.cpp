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

#include <Map.h>

namespace VO {
    void Map::insertKeyFrame(Frame::Ptr frame_)
    {
	if (keyframes.find(frame_->id) == keyframes.end())
	{
	    keyframes.insert(make_pair(frame_->id, frame_));
	}
	else
	{
	    keyframes[frame_->id] = frame_;
	}
    }
    
   void Map::insertPoint3d(Point_3d::Ptr point3d_)
   {  
	if (map.find(point3d_->id) == map.end())
	{
	    map.insert(make_pair(point3d_->id, point3d_));
	}
	else
	{
	    map[point3d_->id] = point3d_;
	}
   }
} // end of VO
