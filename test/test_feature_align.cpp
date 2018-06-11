#include <string>
#include <string.h>
#include <assert.h>
#include <common.h>
#include <FeatureAlignment.h>

using namespace std;
using namespace Eigen;
using namespace cv;


void genRefPatch(const Mat& img_, const Vector2d& px_, Mat& ref_patch_with_border_)
{
  // compute interpolation weights
  const int u_r = floor(px_[0]);
  const int v_r = floor(px_[1]);
  const float subpix_x = px_[0] - u_r;
  const float subpix_y = px_[1] - v_r;
  const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
  const float wTR = subpix_x*(1.0-subpix_y);
  const float wBL = (1.0-subpix_x)*subpix_y;
  const float wBR = subpix_x*subpix_y;
  
  //
  ref_patch_with_border_ = Mat(10,10,CV_8UC1);
  uint8_t* it_patch = ref_patch_with_border_.data;
  const int stride = img_.step.p[0];
  for (int y=0; y<10; y++)
  {
    uint8_t* it = (uint8_t*)img_.data + (v_r+y-4-1)*stride + u_r-4-1;
    for (int x=0; x<10;x++,it++,it_patch++)
      *it_patch = wTL*it[0] + wTR*it[1] + wBL*it[stride] + wBR*it[stride+1];
  }
}

int main(int argc, char** argv)
{
  
  string filename = "test1.png";
  Mat img(imread(filename, 0));
  assert(img.type() == CV_8UC1);
  
  Vector2d px_true(200.3981, 196.2534);
  
  Mat ref_patch_with_border;
  genRefPatch(img, px_true, ref_patch_with_border);
  
  uint8_t* ref_match = NULL;
  posix_memalign((void**)&ref_match, 16, 64);
  uint8_t* it_ref_patch = ref_match;
  for (int y=1; y<9; y++)
  {
    uint8_t* it = ref_patch_with_border.data + y*10 + 1;
    for (int x=0; x<8; x++,it++,it_ref_patch++)
    {
      *it_ref_patch = *it;
    }
  }
  
  Vector2d px_est, px_err(-1.1,0.8);
  double h_inv;
  for (int i=0;i<1000;i++)
  {
    px_est = px_true-px_err;
    Vector2f dir = px_err.normalized().cast<float>();
    VO::featureAlign1D(img, dir, ref_patch_with_border.data,ref_match,15,px_est,h_inv);
    
  }
  
  Vector2d e = px_est-px_true;
  cout << "error after 1000 iters: " << e <<endl;
  
  for (int i=0; i<1000; i++)
  {
    px_est = px_true-px_err;
    VO::featureAlign2D(img,ref_patch_with_border.data,ref_match,15,px_est);
    
  }
  e = px_est-px_true;
  cout << "error after 1000 iters: " << e <<endl;
  return 0;
}