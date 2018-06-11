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

#include <algorithm>
#include <ImageAlignment.h>
#include <Frame.h>
#include <Feature.h>
#include <Param.h>
#include <Point3d.h>
#include <Camera.h>
#include <math_funs.h>

namespace VO {
  
ImageAlignment::ImageAlignment(int n_levels_, int min_level_, int n_iter_, OptimizeMethod method_, bool display_, bool verbose_) : 
  max_level(n_levels_), min_level(min_level_),
  display(display_)
{
  n_iter = n_iter_;
  n_iter_init = n_iter;
  method = method_;
  verbose = verbose_;
  eps = 1e-6;
}

size_t ImageAlignment::runOpt(FramePtr ref_frame_, FramePtr cur_frame_)
{
  reset();
  
  if (ref_frame_->features.empty())
  {
    std::cout << "No features to track!" << std::endl;
    return 0;
  }
  
  ref_frame = ref_frame_;
  cur_frame = cur_frame_;
  
  ref_patch_cache = cv::Mat(ref_frame->features.size(), patch_area, CV_32F);
  jac.resize(Eigen::NoChange, ref_patch_cache.rows*patch_area);
  visible_fts.resize(ref_patch_cache.rows, false);
  
  SE3 Tcfr(cur_frame->Tw2c * ref_frame->Tw2c.inverse());
  
  for (level = max_level; level>=min_level; --level)
  {
    mu = 0.1;
    jac.setZero();
    have_ref_patch_cache = false;
    if (verbose)
      std::cout << "pyramid level: " << level << std::endl;
    optimize(Tcfr);
  }
  cur_frame->Tw2c = Tcfr * ref_frame->Tw2c;
  
  return n_meas/patch_area;
}

Matrix< double, 6 , 6  > ImageAlignment::getFisherInformationMatrix()
{
  double sigma = 5e-4*255*255;
  Matrix<double, 6, 6> I = H/sigma;
  return I;
}

void ImageAlignment::preComputeRefPatches()
{
  const int border = patch_halfsize+1;
  const cv::Mat& ref_img = ref_frame->img_pyr.at(level);
  const int stride = ref_img.cols;
  const float img_scale = 1.0f/(1<<level);
  const Vector3d ref_pos = ref_frame->pos();
  const double focal_length = ref_frame->camera->fx;
  size_t feature_counter = 0;
  std::vector<bool>::iterator it_visibility = visible_fts.begin();
  for (auto it = ref_frame->features.begin(); it!=ref_frame->features.end();++it,++feature_counter,++it_visibility)
  {
    const float u_ref = (*it)->p2d[0]*img_scale;
    const float v_ref = (*it)->p2d[1]*img_scale;
    const int u_ref_r = floor(u_ref);
    const int v_ref_r = floor(v_ref);
    
    if ((*it)->point == NULL || u_ref_r-border<0 || v_ref_r-border<0 || u_ref_r+border>=ref_img.cols || v_ref_r+border>=ref_img.rows)
    {
      continue;
    }
    
    *it_visibility = true;
    const double depth(((*it)->point->pos - ref_pos).norm());
    const Vector3d xyz_ref((*it)->pu*depth);
    
    // projection jac
    Matrix<double, 2, 6> frame_jac;
    Frame::jacobian_xyz2uv(xyz_ref, frame_jac);
    
    // computer pixel interpolation
    const float subpix_x = u_ref-u_ref_r;
    const float subpix_y = v_ref-v_ref_r;
    const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
    const float wTR = subpix_x*(1.0-subpix_y);
    const float wBL = (1.0-subpix_x)*subpix_y;
    const float wBR = subpix_x*subpix_y;
    size_t pixel_counter = 0;
    float* it_cache = reinterpret_cast<float*>(ref_patch_cache.data) + patch_area*feature_counter;
    for (int y=0; y<patch_size; ++y)
    {
      uint8_t* it_ref = (uint8_t*)ref_img.data + (v_ref_r+y-patch_halfsize)*stride + (u_ref_r-patch_halfsize);
      for (int x=0; x<patch_size; ++x,++it_ref,++it_cache,++pixel_counter)
      {
	*it_cache = wTL*it_ref[0] + wTR*it_ref[1] + wBL*it_ref[stride] + wBR*it_ref[stride+1];
	// compute gradient of warped image
	float dx = 0.5*((wTL*it_ref[1] + wTR*it_ref[2] + wBL*it_ref[stride+1] + wBR*it_ref[stride+2])
	   -(wTL*it_ref[-1] + wTR*it_ref[0] + wBL*it_ref[stride-1] + wBR*it_ref[stride])
	);
	float dy = 0.5*((wTL*it_ref[stride] + wTR*it_ref[stride+1] + wBL*it_ref[stride*2] + wBR*it_ref[stride*2+1])
	   -(wTL*it_ref[-stride] + wTR*it_ref[1-stride] + wBL*it_ref[0] + wBR*it_ref[1])
	);
	
	// compute jacobian
	jac.col(feature_counter*patch_area + pixel_counter) = (dx*frame_jac.row(0) + dy*frame_jac.row(1))*img_scale;
      }
    }
  }
  
  have_ref_patch_cache = true;
}
// compute hessian matrix and residual error
double ImageAlignment::computeResiduals(const SE3& model_, bool linearize_system_, bool compute_weight_scale_)
{
  // 
  const cv::Mat& cur_img = cur_frame->img_pyr.at(level);
  
  //if (linearize_system_ )
  
  if (!have_ref_patch_cache)
    preComputeRefPatches();
  
  // compute weights
  std::vector<float> errors;
  if (compute_weight_scale_)
    errors.reserve(visible_fts.size());
  const int stride = cur_img.cols;
  const int border = patch_halfsize+1;
  const float img_scale = 1.0f/(1<<level);
  const Vector3d ref_pos(ref_frame->pos());
  float chi2 = 0.0f;
  size_t feature_counter = 0;
  std::vector<bool>::iterator it_visibility = visible_fts.begin();
  for (auto it=ref_frame->features.begin(); it!=ref_frame->features.end(); ++it,++feature_counter,++it_visibility)
  {
    if (!(*it_visibility))
      continue;
    
    // compute pixel loc
    const double depth = ((*it)->point->pos - ref_pos).norm();
    const Vector3d xyz_ref((*it)->pu*depth);
    const Vector3d xyz_cur(model_ * xyz_ref);
    const Vector2f uv_cur_pyr(cur_frame->camera->world2pixel(xyz_cur, cur_frame->Tw2c).cast<float>()* img_scale);
    const float u_cur = uv_cur_pyr[0];
    const float v_cur = uv_cur_pyr[1];
    const int u_cur_r = floor(u_cur);
    const int v_cur_r = floor(v_cur);
    
    // check border
    if (u_cur_r < 0 || v_cur_r<0 || u_cur_r-border<0 || v_cur_r-border<0 || u_cur_r+border>=cur_img.cols || v_cur_r+border>=cur_img.rows)
    {
      continue;
    }
    // compute interpolation
    const float subpix_x = u_cur-u_cur_r;
    const float subpix_y = v_cur-v_cur_r;
    const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
    const float wTR = subpix_x*(1.0-subpix_y);
    const float wBL = (1.0-subpix_x)*subpix_y;
    const float wBR = subpix_x*subpix_y;
    float* it_ref_patch = reinterpret_cast<float*>(ref_patch_cache.data) + patch_area*feature_counter;
    int pixel_counter = 0;
    for (int y=0; y<patch_size; ++y)
    {
      uint8_t* it_cur = (uint8_t*)cur_img.data + (v_cur_r+y-patch_halfsize)*stride + (u_cur_r-patch_halfsize);
      for (int x=0; x<patch_size; ++x,++pixel_counter,++it_cur,++it_ref_patch)
      {
	// compute error
	const float intensity_cur = wTL*it_cur[0] + wTR*it_cur[1] + wBL*it_cur[stride] + wBR*it_cur[stride+1];
	const float err = intensity_cur - (*it_ref_patch);
	
	if (compute_weight_scale_)
	  errors.push_back(fabs(err));
	
	// robust
	float weight = 1.0;
	if (using_weight)
	{
	  weight = weightFunction->value(err/scale);
	}
	
	chi2 += err*err*weight;
	n_meas++;
	
	if (linearize_system_)
	{
	  const Vector6d J(jac.col(feature_counter*patch_area + pixel_counter));
	  H.noalias() += J*J.transpose()*weight;
	  Jerr.noalias() -= J*err*weight;
	}
      }
    }
  }
  
  if (compute_weight_scale_ && iter == 0)
    scale = scaleEstimator->compute(errors);
  
  return chi2/n_meas;
}

int ImageAlignment::solve()
{
  dX = H.ldlt().solve(Jerr);
  if ((bool)std::isnan((double)dX[0]))
    return 0;
  return 1;
}

void ImageAlignment::update(const SE3& old_model_, SE3& new_model_)
{
  new_model_ = old_model_ * SE3::exp(-dX);
}


void ImageAlignment::startIteration()
{

}

void ImageAlignment::finishIteration()
{
  
}




  
} // end of VO