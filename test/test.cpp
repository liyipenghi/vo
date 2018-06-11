#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <utils.h>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;

#include <FeatureDetector.h>
#include <Frame.h>
#include <Feature.h>

int main()
{
    VO::Frame* frame;
    VO::Features fts;
        
    string name = "00000.jpg";

    cv::Mat img = cv::imread(name,0);
    VO::FastDetector detector(img.rows,img.cols,30,3);
    vector<cv::Mat> img2;
    
    image_utils::createImgPyr(img,img2,3);
    
    detector.detect(frame,img2,5.0,fts);
    
    cout << "number of features: " << fts.size() << endl;
    
    cv::Mat out;
    vector<cv::KeyPoint> kpts;
    for (VO::Feature* fs : fts)
    {
	cv::KeyPoint kpt;
	kpt.pt.x = fs->p2d(0,0);
	kpt.pt.y = fs->p2d(1,0);
	kpts.push_back(kpt);
    }
    cv::drawKeypoints(img,kpts,out,cv::Scalar(0,1,0,1));
    
    cv::imwrite("kp.png",out);

    return 0;
}