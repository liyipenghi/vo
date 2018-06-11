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

#ifndef FEATRUE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <common.h>
#include <Frame.h>

namespace VO
{
    // struct IntrestPoint intrinst
    struct Corner {
        float x;
        float y;
        int level;
        float score;
        float angle;
        Corner(float x_, float y_, int level_, float score_, float angle_) :
                x(x_), y(y_), level(level_), score(score_), angle(angle_)
        {} 
    };
    typedef std::vector<Corner> Corners;
    // base class of feature detector
    class BaseDetector {
    public:
        BaseDetector(const int width_, const int height_, const int cell_size_, const int level_);
        virtual ~BaseDetector() {};

        virtual void detect(Frame* frame_, const ImgPyr& img_pyr_, const double thresh_, Features& features_) = 0;
        void setGridOccupancy(const Vector2d& px_);
        void setExistingFeatures(const Features& fts_);
    protected:
        static const int border = 8;
        const int cell_size;
        const int level;
        const int grid_cols;
        const int grid_rows;
        std::vector<bool> grid_occupancy;

        void resetGrid();

        inline int getCellIndex(int x_, int y_, int level_)
        {
            const int scale = (1<<level_);
            return (scale*y_)/cell_size*grid_cols + (scale*x_)/cell_size;
        }
    };
    typedef boost::shared_ptr<BaseDetector> FtrDetector;
    // fast feature detector
    class FastDetector : public BaseDetector {
    public:
        FastDetector(const int width_, const int height_, const int cell_size_, const int level_);
        virtual ~FastDetector() {}

        virtual void detect(Frame* frame_, const ImgPyr& img_pyr_, const double thresh_, Features& features_);
    };
}// namspace vo

#endif
