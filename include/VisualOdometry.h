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

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <common.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Map.h>

namespace VO {

  class VisualOdometry {
    enum STATE
    {
	INITIAL,
        FIRST_TRACKING,
        SECEND_TRACKING,
        COMMON_STATE,
        LOST_TRACKING
    };
  private:
      STATE working_state;
      /* 地图 */
      Map::Ptr map;
      /* 关键帧 */
      Frame::Ptr cur_frame;
      Frame::Ptr ref_frame;
      /* 特征检测 */
      cv::Ptr<cv::ORB> orb;
      std::vector<cv::KeyPoint> kpts_curr;
      cv::Mat desc_curr;
      /* 特征匹配 */
      cv::FlannBasedMatcher matcher_flann;
      vector<Point_3d::Ptr> matched_point3ds;
      vector<int> matched_point2ds_index;
      
      SE3 T_c_w; // estimated pose
      int num_linliers;
      int num_lost;
      
      /* 参数 */
      int num_features;
      double scale;
      int level;
      float match_ratio;
      int max_lost;
      int min_inliers;
      double min_rot;
      double min_trans;
      double map_erase_ratio;
  public:
      VisualOdometry();
      ~VisualOdometry();
      
      bool addFrame(Frame::Ptr frame_);
      
  protected:
      void detectFeature();
      void extractDesc();
      void match();
      void poseEstimation();
      void optMap();
      void addKeyFrame();
      void addPoint3d();
      bool checkEstimatedPose();
      bool checkKeyFrame();
  }; // end of VisualOdometry

} // end fo namespace VO

#endif
