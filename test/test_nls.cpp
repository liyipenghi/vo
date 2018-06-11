#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/StdVector>
#include <NLSSolver.h>
#include <assert.h>

using namespace std;
using namespace Eigen;

// y=2*x^2
namespace Vision{

class TestNls : public NLSSolver<2, Vector2d>
{
protected:
  double eps;
  int n_iter_;
  
  // 
  vector<double> ys;
  vector<double> xs;
  
  virtual double computeResiduals(const Vector2d& model_, bool linearize_system_, bool compute_weight_scale_ = false);
  virtual int solve();
  virtual void update(const Vector2d& old_model_, Vector2d& new_model_);
  virtual void startIteration();
  virtual void finishIteration();
public:
  void runOpt(Vector2d& model_);
  TestNls();
  ~TestNls(){};
};

TestNls::TestNls() :
  eps(1e-10), n_iter_(20)
{
  method = GaussNewton;
  n_iter = n_iter_;
  verbose = true;
  
  xs.push_back(1);
  ys.push_back(1-0.001 + 2);
  
  xs.push_back(2);
  ys.push_back(4+0.0002 + 2);
  
  xs.push_back(3);
  ys.push_back(9+0.003 + 2);
  
  xs.push_back(4);
  ys.push_back(16-0.0053 + 2);
  
  xs.push_back(5);
  ys.push_back(25-0.04 + 2);
}

double TestNls::computeResiduals(const Vector2d& model_, bool linearize_system_, bool compute_weight_scale_)
{
  double chi2 = 0.0;
  size_t num = 0;
  
  for (int i=0; i< ys.size(); i++)
  {
    // compute error
    float res = ys[i] - model_(0,0)*xs[i]*xs[i] - model_(1,0);
    chi2 = res*res;
    num++;
    
    if (linearize_system_)
    {
      Matrix<double, 2, 1> jac;
      jac(0,0) = -xs[i]*xs[i];
      jac(1,0) = -1;
      
      H += jac*jac.transpose();
      Jerr += jac*res;
    }
  }
  chi2 /= num;
  
  return chi2;
}

int TestNls::solve()
{
  dX = H.ldlt().solve(-Jerr);
  if ((bool) std::isnan((double) dX[0]))
    return 0;
  return 1;
}

void TestNls::update(const Vector2d& old_model_, Vector2d& new_model_)
{
  new_model_ = dX + old_model_;
}

void TestNls::startIteration()
{
  
}

void TestNls::finishIteration()
{

}

void TestNls::runOpt(Vector2d& model_)
{
  optimize(model_);
}

}

int main(int argc, char** argv)
{
  Vision::TestNls test;
  
  Vector2d x;
  x(0,0) = 0.5;
  x(1,0) = -1.0;
  test.setRobustCostFunction(Vision::TestNls::TDistScale,Vision::TestNls::HuberWeight);
  test.runOpt(x);
  
  cout << "result: " << x << endl;
  
  return 0;
}



