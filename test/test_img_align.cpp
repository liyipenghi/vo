#include <Camera.h>
#include <iostream>
#include <FeatureDetector.h>
#include <ImageAlignment.h>
#include <Frame.h>
#include <Point3d.h>
#include <Feature.h>
#include <Param.h>

class TestImgAlign {
public:
  TestImgAlign();
  virtual ~TestImgAlign() {};
  
  void testSequence(const std::string& data_dir, const std::string& exp_name,
    VO::FeatureDetector* featureDetector
  );
  VO::PinholeCamera* camera;
  VO::FramePtr cur_frame;
  VO::FramePtr ref_frame;
};

TestImgAlign::TestImgAlign() : 
  camera(new VO::PinholeCamera(752,480,315.5,315.5,376.0,24.0))
{

}

void TestImgAlign::testSequence(const std::string& data_dir, const std::string& exp_name, VO::FeatureDetector* featureDetector)
{
  
}
