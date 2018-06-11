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

#ifndef IMAGE_ALIGNMENT_H
#define IMAGE_ALIGNMENT_H

#include <NLSSolver.h>
#include <common.h>

namespace VO {
  
  class PinholeCamera;
  class Feature;
  // model : SE3
  class ImageAlignment : public Vision::NLSSolver<6, SE3>
  {
    static const int patch_halfsize = 2;
    static const int patch_size = 4;
    static const int patch_area = 16;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // 残差图像
    cv::Mat err_img;
    ImageAlignment(int n_levels_, int min_level_, 
		   int n_iter_, OptimizeMethod method_,
		   bool display_, bool verbose_
		  );
    size_t runOpt(FramePtr ref_frame_, FramePtr cur_frame_);
    Matrix<double, 6, 6> getFisherInformationMatrix();
    
  protected:
    FramePtr ref_frame;
    FramePtr cur_frame;
    int level;
    bool display;
    int max_level;
    int min_level;
    
    // intermedia variables
    Matrix<double, 6, Dynamic, ColMajor> jac;
    bool have_ref_patch_cache;
    cv::Mat ref_patch_cache;
    std::vector<bool> visible_fts;
    
    void preComputeRefPatches();
    virtual double computeResiduals(const SE3& model_, bool linearize_system_, bool compute_weight_scale_=false);
    virtual int solve();
    virtual void update(const Model& old_model_, Model& new_model_);
    virtual void startIteration();
    virtual void finishIteration();
  };
  
} // end of 

#endif