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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include <Param.h>
#include <VisualOdometry.h>

namespace VO {

using namespace cv;
  
VisualOdometry::VisualOdometry():
  working_state(INITIAL),
  ref_frame(nullptr),
  cur_frame(nullptr),
  map(new Map),
  num_lost(0),
  num_linliers(0),
  matcher_flann(new cv::flann::LshIndexParams(5, 10, 2))
{
    num_features = Param::get<int>("number_features");
    scale = Param::get<double>("scale");
    level = Param::get<int>("pyr_level");
    match_ratio = Param::get<float>("match_ratio");
    max_lost = Param::get<int>("max_lost");
    min_inliers = Param::get<int>("min_inliers");
    min_rot = Param::get<double>("min_rotation");
    min_trans = Param::get<double>("min_translation");
    map_erase_ratio = Param::get<double>("map_erase_ratio");
    orb = cv::ORB::create(num_features, scale, level);
}
VisualOdometry::~VisualOdometry()
{
   
}

void VisualOdometry::detectFeature()
{
    boost::timer timer;
    // 仅仅使用图像金字塔的第一层
    orb->detect(cur_frame->img_pyr[0], kpts_curr);
    cout << "ORB feature detection cost time: " << timer.elapsed() << endl;
}

void VisualOdometry::extractDesc()
{
    boost::timer timer;
    orb->compute(cur_frame->img_pyr[0], kpts_curr, desc_curr);
    cout << "Compute descriptor cost time: " << timer.elapsed() << endl;
}

void VisualOdometry::match()
{
    boost::timer timer;
    vector<DMatch> matches;
    Mat desp_map;
    vector<Point_3d::Ptr> candidate;
    for (auto& ps: map->map)
    {
	Point_3d::Ptr p = ps.second;
	// 判断是否在当前帧
	if (cur_frame->isInFrame(p->pos))
	{
	    p->visible_times++;
	    candidate.push_back(p);
	    desp_map.push_back(p->desc);
	}
    }
    matcher_flann.match(desp_map, desc_curr, matches);
    float min_dist = std::min_element(matches.begin(),matches.end(),
      [] (const cv::DMatch& m1, const cv::DMatch& m2)
      {
	return m1.distance < m2.distance;
      }
    ) -> distance;
    
    matched_point3ds.clear();
    matched_point2ds_index.clear();
    for (cv::DMatch& m : matches)
    {
	if (m.distance < max<float>(min_dist*match_ratio, 30.0))
	{
	    matched_point3ds.push_back(candidate[m.queryIdx]);
	    matched_point2ds_index.push_back(m.trainIdx);
	}
    }
}
void VisualOdometry::poseEstimation()
{
    // 3d-2d correspondances
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    
    for (int index:matched_point2ds_index)
    {
	pts2d.push_back(kpts_curr[index].pt);
    }
    for (Point_3d::Ptr pt:matched_point3ds)
    {
	pts3d.push_back(pt->getPoint());
    }
    
    cv::Mat K = ref_frame->camera->cvK;
    Mat rvec,tvec,inliers;
    cv::solvePnPRansac(pts3d,pts2d,K,Mat(),rvec,tvec,false,100,4.0,0.99,inliers);
    num_linliers = inliers.rows;
    cout << "inliers: " << num_linliers << endl;
}

} // end of VO