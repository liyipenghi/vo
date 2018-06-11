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

#ifndef FEATURE_ALIGNMENT_H
#define FEATURE_ALIGNMENT_H

#include <common.h>

namespace VO {
  // implement of inverse-composition
  bool featureAlign1D(const cv::Mat& cur_img,
     const Vector2f& dir_,
     uint8_t* ref_patch_with_border_,
     uint8_t* ref_patch_,
     const int n_iter_,
     Vector2d& cur_px_estimated_,
     double& h_inv_
  );
  bool featureAlign2D(const cv::Mat& cur_img_,
      uint8_t* ref_patch_with_border_,
      uint8_t* ref_patch_,
      const int n_iter_,
      Vector2d& cur_px_estimated_
  );
  
} // end of VO

#endif