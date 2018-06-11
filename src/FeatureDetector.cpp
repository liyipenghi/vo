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

#include <FeatureDetector.h>
#include <Feature.h>
#include <opencv2/features2d.hpp>
#include <utils.h>
#include <vector>
#include <math.h>
#include <fast/fast.h>

namespace VO {
    /* abstract class for feature detector */
    BaseDetector::BaseDetector(const int width_,
            const int height_, const int cell_size_,
            const int level_) :
        cell_size(cell_size_),
        level(level_),
        grid_cols(ceil(static_cast<double>(width_)/cell_size)),
        grid_rows(ceil(static_cast<double>(height_)/cell_size)),
        grid_occupancy(grid_rows*grid_cols, false)
    {}
    void BaseDetector::resetGrid()
    {
        std::fill(grid_occupancy.begin(), grid_occupancy.end(), false);
    }
    // left empty
    void BaseDetector::setGridOccupancy(const Vector2d& px_)
    {
        std::vector<int> x;
        cv::Mat img;
        
    }
    void BaseDetector::setExistingFeatures(const Features& fts_)
    {}

    /* fast corner detector */
    FastDetector::FastDetector(const int width_, 
                                const int height_, 
                                const int cell_size_, 
                                const int level_) : 
        BaseDetector(width_,height_,cell_size_,level_)
        {}

    void FastDetector::detect(Frame* frame_, 
                         const ImgPyr& img_pyr_, 
                         const double thresh_, 
                         Features& features_)
    {
        Corners corners(grid_cols*grid_rows, Corner(0.0f,0.0f,0,thresh_,0.0f));

        for (int L=0; L<level; L++)
        {
            const int scale = (1<<L);
	    /*
	    vector<fast::fast_xy> fast_corners;
	    
	    fast::fast_corner_detect_10(
	      (fast::fast_byte*)img_pyr_[L].data, img_pyr_[L].cols,
	      img_pyr_[L].rows, img_pyr_[L].cols, thresh_, fast_corners);
	    
	    vector<int> scores, nm_corners;
	    fast::fast_corner_score_10((fast::fast_byte*)img_pyr_[L].data, img_pyr_[L].cols, fast_corners, 20, scores);
	    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);
	    */
             vector<cv::KeyPoint> kpts;
             cv::FAST(img_pyr_[L], kpts, thresh_, true);
            for (auto it=kpts.begin(); it!=kpts.end(); it++)
            {
                cv::KeyPoint kpt = *it;
                //fast::fast_xy kpt = fast_corners.at(*it);
		const int k = static_cast<int>((kpt.pt.y*scale)/cell_size)*grid_cols
                            + static_cast<int>((kpt.pt.x*scale)/cell_size);
                if (grid_occupancy[k])
                    continue;
                const float score = image_utils::shiTomasiScore(img_pyr_[L], kpt.pt.x, kpt.pt.y);
                //cout << "score: " << score << " " << corners.at(k).score << endl;
		if (score > corners.at(k).score)
                    corners.at(k) = Corner(kpt.pt.x*scale, kpt.pt.y*scale, L, score, 0.0f);
            }
        }
        
        //cout << "total num: " << corners.size() << endl;
	for (const Corner& c : corners)
	{
	    if (c.score > thresh_)
	    {
		features_.push_back(new Feature(frame_,Vector2d(c.x, c.y), c.level));
	    }
	}
	/*
        std::for_each(corners.begin(), corners.end(), [&](Corner& c)
        {
            if (c.score > thresh_)
            {
                features_.push_back(new Feature(frame_, Vector2d(c.x, c.y), c.level));
            }
        });
	*/
        resetGrid();
    }
} // namespace VO
