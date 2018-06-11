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

#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>

#include <Param.h>
#include <VisualOdometry.h>

using namespace std;

int main(int argc, char** argv)
{
    if (argc!=2)
    {
       cout<<"test_vo para.ymal"<<endl;
       return 1;
    }
    
    VO::Param::setParametersFile(argv[1]);
    VO::VisualOdometry vo;
    
    string img_dir = VO::Param::get<string>("dataset_dir");
    
    cout << "dir of images: " << img_dir <<endl;
    
    ifstream fin(img_dir+"/associate.txt");
    if (!fin)
    {
	cout << "No associate files found!" << endl;
	return 1;
    }
    
    vector<string> rgbfiles, depthfiles;
    vector<double> rgbtimes, depthtimes;
    
    while(!fin.eof())
    {
	    string rgbtime, rgbfile, depthtime, depthfile;
	    fin >> rgbtime >> rgbfile >> depthtime >> depthfile;
	    rgbtimes.push_back(atof(rgbtime.c_str()));
	    rgbfiles.push_back(rgbfile);
	    depthtimes.push_back(atof(depthtime.c_str()));
	    depthfiles.push_back(depthfile);
	
	if (fin.good() == false)
	  break;
    }
    
   // VO::PinholeCamera camera;
    
    cout << "total image: " << rgbfiles.size() << endl;
    for (int i=0;i<rgbfiles.size();i++)
    {
	cout << "loop " << i << endl;
	cv::Mat color = cv::imread(rgbfiles[i]);
	cv::Mat depth = cv::imread(depthfiles[i]);
	if (color.data==nullptr || depth.data==nullptr)
	  break;
	
	VO::Frame frame;
	
    }
    
    return 0;
}