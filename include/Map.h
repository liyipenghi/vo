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

#ifndef MAP_H
#define MAP_H

#include <common.h>
#include <Frame.h>
#include <Point3d.h>

namespace VO {
  
  class Map {
  public:
    typedef boost::shared_ptr<Map> Ptr;
    
    unordered_map<unsigned long, Point_3d::Ptr> map; // global map
    unordered_map<unsigned long, Frame::Ptr> keyframes; // total keyframes
 
    Map() {}
    void insertKeyFrame(Frame::Ptr frame_);
    void insertPoint3d(Point_3d::Ptr point3d_);
  };
  
} // end of VO

#endif