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

#ifndef ROBUST_COST_H
#define ROBUST_COST_H

#include <vector>
#include <stdlib.h>
#include <memory>

namespace Vision {
  using namespace std;
  // interface for scale estimator
  class ScaleEstimator
  {
  public:
    virtual ~ScaleEstimator() {};
    virtual float compute(std::vector<float>& errors) const = 0;
  };
  typedef std::shared_ptr<ScaleEstimator> ScaleEstimatorPtr;
  
  class UnitScaleEstimator : public ScaleEstimator
  {
  public:
    UnitScaleEstimator() {}
    virtual ~UnitScaleEstimator() {}
    virtual float compute(std::vector<float>& errors) const {return 1.0f;};
  };
  // estimate scale by fitting a t-distribution to the data with given dof
  class TDistributionScaleEstimator : public ScaleEstimator
  {
  public:
    TDistributionScaleEstimator(const float dof_ = DEFAULT_DOF);
    virtual ~TDistributionScaleEstimator(){};
    virtual float compute(std::vector<float>& errors) const;
    
    static const float DEFAULT_DOF;
    static const float INITIAL_SIGMA;
  protected:
    float dof;
    float initial_sigma;
  };
  // estimate scale by computing the median absolute deviation
  class MADScaleEstimator : public ScaleEstimator
  {
  public:
    MADScaleEstimator() {};
    virtual ~MADScaleEstimator() {};
    virtual float compute(vector<float>& errors) const;
  private:
    static const float NORMALIZER;
  };
  // estimate scale by computing the standard deviation
  class NormalDistributionScaleEstimator : public ScaleEstimator
  {
  public:
    NormalDistributionScaleEstimator() {};
    virtual ~NormalDistributionScaleEstimator() {};
    virtual float compute(std::vector<float>& errors) const;
  };
  /* interface for weight function */
  class WeightFunction
  {
  public:
    virtual ~WeightFunction() {};
    virtual float value(const float& x) const = 0;
    virtual void configure(const float& param) {};
  };
  typedef std::shared_ptr<WeightFunction> WeightFunctionPtr;
  
  class UnitWeightFunction : public WeightFunction
  {
  public:
    UnitWeightFunction() {};
    virtual ~UnitWeightFunction() {}; 
    virtual float value(const float& x) const {return 1.0f;};
  };
  // Tukey's hard re-decending function
  class TukeyWeightFunction : public WeightFunction
  {
  public:
    TukeyWeightFunction(const float b = DEFAULT_B);
    virtual ~TukeyWeightFunction() {};
    virtual float value(const float& x) const;
    virtual void configure(const float& param);
    
    static const float DEFAULT_B;
  private:
    float b2;
  };
  class TDistributionWeightFunction : public WeightFunction
  {
  public:
    TDistributionWeightFunction(const float dof_ = DEFAULT_DOF);
    virtual ~TDistributionWeightFunction() {};
    virtual float value(const float& x) const;
    virtual void configure(const float& param);
    
    static const float DEFAULT_DOF;
  private:
    float dof;
    float normalizer;
  };
  
  class HuberWeightFunction : public WeightFunction
  {
  public:
    HuberWeightFunction(const float k = DEFAULT_K);
    virtual ~HuberWeightFunction() {};
    virtual float value(const float& x) const;
    virtual void configure(const float& param);
    
    static const float DEFAULT_K;
  private:
    float k;
  };
} // end of namespace Vision

#endif