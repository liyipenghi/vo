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

#include <numeric>
#include <math.h>
#include <algorithm>
#include <assert.h>
#include <RobustCost.h>

namespace Vision {
  
  const float TDistributionScaleEstimator::INITIAL_SIGMA = 5.0f;
  const float TDistributionScaleEstimator::DEFAULT_DOF = 5.0f;
  
TDistributionScaleEstimator::TDistributionScaleEstimator(const float dof_) :
  dof(dof_), initial_sigma(INITIAL_SIGMA)
{}
// t分布（学生分布）的方差计算，并返回标准差倒数
float TDistributionScaleEstimator::compute(std::vector< float >& errors) const
{
  float initial_lambda  = 1.0f / (initial_sigma * initial_sigma);
  int num = 0;
  float lambda = initial_lambda;
  int iterations = 0;
  while (std::abs(lambda - initial_lambda) > 1e-3)
  {
     iterations++;
     initial_lambda = lambda;
     num = 0;
     lambda = 0;
     for (auto it = errors.begin(); it != errors.end(); it++)
     {
	num++;
	const float error2 = (*it)*(*it);
	lambda += error2 * ((dof + 1.0f) / (dof + initial_lambda * error2));
     }
     lambda = float(num) / lambda;
  }
  
  return sqrt(1.0f / lambda);
}

const float MADScaleEstimator::NORMALIZER = 1.48f; // 1/0.6745
// 取中间的值
float MADScaleEstimator::compute(vector< float >& errors) const
{
  assert(!errors.empty());
  typename vector<float>::iterator it = errors.begin() + floor(errors.size()/2);
  std::nth_element(errors.begin(), it, errors.end());
  return (*it)*NORMALIZER;
}
// 正态分布
float NormalDistributionScaleEstimator::compute(std::vector< float >& errors) const
{
  const float mean = std::accumulate(errors.begin(), errors.end(), 0)/errors.size();
  float var = 0.0f;
  std::for_each(errors.begin(), errors.end(), [&](const float d)
  {
    var += (d-mean)*(d-mean);
  });
  return sqrt(var);
}

const float TukeyWeightFunction::DEFAULT_B = 4.6851f;

TukeyWeightFunction::TukeyWeightFunction(const float b)
{
  configure(b);
}

float TukeyWeightFunction::value(const float& x) const
{
  const float x2 = x*x;
  if (x2 <= b2)
  {
    const float tmp = 1.0f - x2/b2;
    return tmp*tmp;
  }
  else
  {
    return 0.0f;
  }
}

void TukeyWeightFunction::configure(const float& param)
{
  b2 = param*param;
}

const float TDistributionWeightFunction::DEFAULT_DOF = 5.0f;
TDistributionWeightFunction::TDistributionWeightFunction(const float dof_)
{
  configure(dof_);
}

float TDistributionWeightFunction::value(const float& x) const
{
  return ((dof + 1.0f) / (dof + x * x));
}

void TDistributionWeightFunction::configure(const float& param)
{
  dof = param;
  normalizer = dof / (dof + 1.0f);
}

const float HuberWeightFunction::DEFAULT_K = 1.345f;

HuberWeightFunction::HuberWeightFunction(const float k)
{
  configure(k);
}

void HuberWeightFunction::configure(const float& param)
{
  k = param;
}

float HuberWeightFunction::value(const float& x) const
{
  const float t_abs = std::abs(x);
  if (t_abs < k)
  {
    return 1.0f;
  }
  else
  {
    return k / t_abs;
  }
}  
} // end of Vision
