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

#include <FeatureAlignment.h>

namespace VO {
  
  bool featureAlign1D(const cv::Mat& cur_img, const Vector2f& dir_, uint8_t* ref_patch_with_border_, uint8_t* ref_patch_, const int n_iter_, Vector2d& cur_px_estimated_, double& h_inv_)
  {
     const int halfpatch_size = 4;
     const int patch_size = 8;
     const int patch_area = 64;
     bool converged = false;
     
     // 
     float __attribute__((__aligned__(16))) ref_patch_dv[patch_area];
     // Hessian matrix
     Matrix2f H;
     H.setZero();
     
     // compute Hessian and Gradient
     const int ref_step = patch_size + 2;
     float* it_dv = ref_patch_dv;
     for (int j=0; j<patch_size; j++)
     {
	uint8_t* it = ref_patch_with_border_ + (j+1)*ref_step + 1;
	for (int i=0; i<patch_size; i++,it++,it_dv++)
	{
	   Vector2f J;
	   J[0] = 0.5*(dir_[0]*(it[1]-it[-1]) + dir_[1]*(it[ref_step]-it[-ref_step]));
	   J[1] = 1;
	   *it_dv = J[0];
	   H += J*J.transpose();
	}
     }
     h_inv_ = 1.0/H(0,0)*patch_area;
     Matrix2f Hinv = H.inverse();
     float mean_diff = 0;
     // compute new location
     
     float u = cur_px_estimated_.x();
     float v = cur_px_estimated_.y();
     
     // iteration
     const float min_update_square = 0.03*0.03;
     const int cur_step = cur_img.step.p[0];
     float chi2 = 0;
     Vector2f update;
     update.setZero();
     for (int iter=0; iter<n_iter_; iter++)
     {
	int u_r = floor(u);
	int v_r = floor(v);
	if (u_r<halfpatch_size || v_r<halfpatch_size || u_r>=cur_img.cols-halfpatch_size || v_r>=cur_img.rows-halfpatch_size)
	  break;
	if (isnan(u) || isnan(v))
	  return false;
	
	float subpix_x = u-u_r;
	float subpix_y = v-v_r;
	float wTL = (1.0-subpix_x)*(1.0-subpix_y);
	float wTR = subpix_x*(1.0-subpix_y);
	float wBL = subpix_y*(1.0-subpix_x);
	float wBR = subpix_x*subpix_y;
	
	uint8_t* it_ref = ref_patch_;
	float* it_ref_dv = ref_patch_dv;
	float new_chi2 = 0.0;
	Vector2f Jerr;
	Jerr.setZero();
	for (int j=0;j<patch_size;j++)
	{
	  uint8_t* it = (uint8_t*) cur_img.data + (v_r+j-halfpatch_size)*cur_step + u_r-halfpatch_size;
	  for (int i=0;i<patch_size;i++,it++,it_ref++,it_ref_dv++)
	  {
	    float search_pixel = wTL*it[0]+wTR*it[1]+wBL*it[cur_step]+wBR*it[cur_step+1];
	    float err = search_pixel - *it_ref + mean_diff;
	    Jerr[0] -= err*(*it_ref_dv);
	    Jerr[1] -= err;
	    new_chi2 += err*err;
	  }
	}
	
	if (iter>0 && new_chi2>chi2)
	{
	  u -= update[0];
	  v -= update[1];
	  break;
	}
	
	chi2 = new_chi2;
	update = Hinv*Jerr;
	u += update[0]*dir_[0];
	v += update[0]*dir_[1];
	mean_diff += update[1];
	
	if (update[0]*update[0]+update[1]*update[1] < min_update_square)
	{
	  converged = true;
	  break;
	}
     }
     
     cur_px_estimated_ << u, v;
     return converged;
  }
  // use model
  bool featureAlign2D(const cv::Mat& cur_img_, uint8_t* ref_patch_with_border_, uint8_t* ref_patch_, const int n_iter_, Vector2d& cur_px_estimated_)
  {
    const int halfpatch_size = 4;
    const int patch_size = 8;
    const int patch_area = 64;
    bool converged = false;
    
    float __attribute__((__aligned__(16))) ref_patch_dx[patch_area];
    float __attribute__((__aligned__(16))) ref_patch_dy[patch_area];
    Matrix3f H;
    H.setZero();
    
    // compute gradient and hessian
    const int ref_step = patch_size+2;
    float* it_dx = ref_patch_dx;
    float* it_dy = ref_patch_dy;
    for (int y=0; y<patch_size; y++)
    {
      uint8_t* it = ref_patch_with_border_+(y+1)*ref_step+1;
      for (int x=0; x<patch_size; x++,it++,it_dx++,it_dy++)
      {
	Vector3f jac;
	jac[0] = 0.5*(it[1]-it[-1]);
	jac[1] = 0.5*(it[ref_step]-it[-ref_step]);
	jac[2] = 1;
	*it_dx = jac[0];
	*it_dy = jac[1];
	H += jac*jac.transpose();
      }
    }
    
    Matrix3f Hinv = H.inverse();
    float mean_diff = 0;
    
    // compute pixel location in new image;
    float u = cur_px_estimated_.x();
    float v = cur_px_estimated_.y();
    
    // iteration
    const float min_update_square = 0.03*0.03;
    const int cur_step = cur_img_.step.p[0];
    float chi2 = 0;
    Vector3f update;
    update.setZero();
    
    for (int iter=0; iter<n_iter_; iter++)
    {
      int u_r = floor(u);
      int v_r = floor(v);
      if (u_r<halfpatch_size || v_r<halfpatch_size || u_r>=cur_img_.cols-halfpatch_size || v_r>=cur_img_.rows-halfpatch_size)
	break;
      
      if (isnan(u) || isnan(v))
	return false;
      
      // compute interpolation weights
      float subpix_x = u-u_r;
      float subpix_y = v-v_r;
      float wTL = (1.0-subpix_x)*(1.0-subpix_y);
      float wTR = subpix_x*(1.0-subpix_y);
      float wBL = (1.0-subpix_x)*subpix_y;
      float wBR = subpix_x*subpix_y;
      
      uint8_t* it_ref = ref_patch_;
      float* it_ref_dx = ref_patch_dx;
      float* it_ref_dy = ref_patch_dy;
      Vector3f Jerr;
      Jerr.setZero();
      for (int y=0;y<patch_size;y++)
      {
	uint8_t* it = (uint8_t*)cur_img_.data+(v_r+y-halfpatch_size)*cur_step+u_r-halfpatch_size;
	for (int x=0; x<patch_size; x++,it++,it_ref++,it_ref_dx++,it_ref_dy++)
	{
	  float search_pixel = wTL*it[0]+wTR*it[1]+wBL*it[cur_step]+wBR*it[cur_step+1];
	  float err = search_pixel - *it_ref + mean_diff;
	  Jerr[0] -= err*(*it_ref_dx);
	  Jerr[1] -= err*(*it_ref_dy);
	  Jerr[2] -= err;
	}
      }
      
      //std::cout << "Hinv = " << Hinv 
      //<< "\t Jerr = " << Jerr
	//   << std::endl;
      
      update = Hinv*Jerr;
      u += update[0];
      v += update[1];
      mean_diff += update[2];
      
      if (update[0]*update[0]+update[1]*update[1] < min_update_square)
      {
	converged = true;
	break;
      }
    }
    
    cur_px_estimated_ << u, v;
    return converged;
  }

} // end of VO