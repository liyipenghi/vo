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

#ifndef FEATURE_H
#define FEATURE_H

#include <Frame.h>
#include <Point3d.h>

namespace VO {
    struct RGB {
        unsigned char r;
        unsigned char g;
        unsigned char b;
        RGB() {};
        RGB(unsigned char r_, unsigned char g_, unsigned char b_) : 
            r(r_), g(g_), b(b_)
        {};
    };

    struct Feature {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        Frame* frame;   // corresponding images frame
        Vector2d p2d;   // 2d position of feature in image
        Vector3d p3d;   // 3d position in word frame
        Vector3d pu;    // uint bearing of feature
        int level;      // image pyramid level
        struct RGB rgb;        // rgb value of feature;
        Point_3d* point;
        Feature(Frame* frame_, const Vector2d p2d_, int level_) : 
                frame(frame_), p2d(p2d_), level(level_)
        {}
        Feature() : frame(nullptr) {}
	~Feature(){};

    };
} // namespace VO

#endif