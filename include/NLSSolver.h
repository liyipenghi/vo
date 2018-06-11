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


#ifndef NLS_SOLVER_h
#define NLS_SOLVER_h

#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/StdVector>
#include <math_funs.h>
#include <RobustCost.h>

namespace Vision {
  
  using namespace std;
  using namespace Eigen;
  
  /* dimesion: dim, Type of model: T, SE2, SE3... */
  template <int dim, typename T>
  class NLSSolver {
  public:
    typedef T Model;
    enum OptimizeMethod
    {
      GaussNewton,
      LevenbergMarquardt
    };
    enum ScaleEstimatorType
    {
      UnitScale,
      TDistScale,
      MADScale,
      NormalScale
    };
    // robust cost function
    enum WeightFunctionType
    {
      UnitWeight,
      TDistWeight,
      TukeyWeight,
      HuberWeight
    };
    
  protected:
    Matrix<double, dim, dim> H;  // Hessian matrix
    Matrix<double, dim, 1> Jerr; // jacobian error
    Matrix<double, dim, 1> dX;	 // update
    bool have_prior;
    Model prior;
    Matrix<double, dim, dim> InformationMatrix; // information matrix
    double chi2;
    double rho;
    OptimizeMethod method;

    // compute
    virtual double computeResiduals(const Model& model_, bool linearized_system_, bool compute_weight_scale_) = 0;
    // solve nonlinear system
    virtual int solve() = 0;
    virtual void update(const Model& old_model, Model& new_model) = 0;
    virtual void applyPrior(const Model& cur_model) {}
    virtual void startIteration() {}
    virtual void finishIteration() {}
    virtual void finishTrial() {}
  public:
    double mu_init, mu;
    double nu_init, nu;
    size_t n_iter_init, n_iter;
    size_t n_trials;
    size_t n_trial_max;
    size_t n_meas;
    bool stop;
    bool verbose;
    double eps;
    size_t iter;
    // robust least squares
    bool using_weight;
    float scale;
    
    ScaleEstimatorPtr scaleEstimator;
    WeightFunctionPtr weightFunction;
    
    NLSSolver();
    virtual ~NLSSolver(){};
    
    // optimizing
    void optimize(Model& model_);
    // Gauss Newton
    void optimizeGaussNewton(Model& model_);
    // Levenberg_Marquardt
    void optimizeLevenbergMarquardt(Model& model_);
    
    // set cost function
    void setRobustCostFunction(ScaleEstimatorType scaleEstimator_, 
      WeightFunctionType weightFunction_
    );
    // set prior
    void setPrior(const Model& prior_,
      const Matrix<double, dim, dim>& information_
    );
    // reset
    void reset();
    // get squared error
    const double& getChi2() const;
    // get information matrix
    const Matrix<double, dim, dim>& getInformationMatrix() const;
  };
  
  
  template<int dim, typename T>
  NLSSolver<dim, T>::NLSSolver() : 
    have_prior(false),
    method(LevenbergMarquardt),
    mu_init(0.01f),
    mu(mu_init),
    nu_init(2.0),
    nu(nu_init),
    n_iter_init(15),
    n_iter(n_iter_init),
    n_trials(0),
    n_trial_max(5),
    n_meas(0),
    stop(false),
    verbose(true),
    eps(1e-10),
    iter(0),
    using_weight(false),
    scale(0.0),
    scaleEstimator(NULL),
    weightFunction(NULL)
  {}
  
  template <int dim, typename T>
  void NLSSolver<dim, T>::optimize(Model& model_)
  {
    if (method == GaussNewton)
      optimizeGaussNewton(model_);
    else if (method == LevenbergMarquardt)
      optimizeLevenbergMarquardt(model_);
  }
  
  template<int dim, typename T>
  void NLSSolver<dim, T>::optimizeGaussNewton(Model& model_)
  {
    // compute weight scale
    if (using_weight)
      computeResiduals(model_, false, true);
    
    // save old model
    Model old_model(model_);
    
    // run iteration
    for (iter = 0; iter < n_iter; iter++)
    {
      rho = 0;
      startIteration();
      
      H.setZero();
      Jerr.setZero();
      // error
      n_meas = 0;
      double new_chi2 = computeResiduals(model_, true, false);
      
      // add prior
      if (have_prior)
	applyPrior(model_);
      
      if (!solve())
      {
	std::cout << "Matrix is colse to singular! Stop," << std::endl;
	std::cout << "H = " << H << std::endl;
	std::cout << "Jerr = " << Jerr << std::endl;
	stop = true;
      }
      
      // check if error increase
      if ((iter > 0 && new_chi2 > chi2) || stop)
      {
	if (verbose)
	{
	  std::cout << "It. " << iter 
		    << "\t Failure"
		    << "\t new_chi2 = " << new_chi2
		    << "\t Error increased. Stop."
		    << std::endl;
	}
	model_ = old_model;
	break;
      }
      
      // update
      Model new_model;
      update(model_, new_model);
      old_model = model_;
      model_ = new_model;
      
      chi2 = new_chi2;
      if (verbose)
      {
	std::cout << "It. " << iter
		  << "\t Success"
		  << "\t new_chi2 = " << new_chi2
		  << "\t n_meas = " << n_meas
		  << "\t x_norm = " << norm_max(dX)
		  << std::endl;
      }
      
      finishIteration();
      
      if (norm_max(dX) < eps)
	break;
    }
  }
  
  template <int dim, typename T>
  void NLSSolver<dim, T>::optimizeLevenbergMarquardt(Model& model_)
  {
    // compute weight
    if (using_weight)
      computeResiduals(model_, false, true);
    // compute error
    chi2 = computeResiduals(model_, true, false);
    
    if (verbose)
      std::cout << "init chi2 = " << chi2
		<< "\t n_meas = " << n_meas
		<< std::endl;
		
    // TODO: compute initial lambda
    // Hartley and Zisserman: "A typical init value of lambda is 10^-3 times the
    // average of the diagonal elements of J'J"
    
    if (mu < 0)
    {
      double H_max_diag = 0;
      double tau = 1e-4;
      for (size_t j=0; j<dim; j++)
	H_max_diag = std::max(H_max_diag, fabs(H(j,j)));
      mu = tau*H_max_diag;
    }
    
    // iteration
    for (iter = 0; iter<n_iter; iter++)
    {
      rho = 0;
      startIteration();
      
      n_trials = 0;
      while (!(rho>0 || stop))
      {
	Model new_model;
	double new_chi2 = -1;
	H.setZero();
	Jerr.setZero();
	
	// initial error
	n_meas = 0;
	computeResiduals(model_, true, false);
	
	// damping term
	H += (H.diagonal() * mu).asDiagonal();
	// add prior
	if (have_prior)
	  applyPrior(model_);
	// solve
	if (solve())
	{
	  // update
	  update(model_, new_model);
	  // new error
	  n_meas = 0;
	  new_chi2 = computeResiduals(new_model, false, false);
	  rho = chi2 - new_chi2;
	}
	else
	{
	  std::cout << "Matrix is close to singular!" << std::endl;
	  std::cout << "H = " << H << std::endl;
	  std::cout << "Jerr = " << Jerr << std::endl;
	  rho = -1;
	}
	
	if (rho > 0)
	{
	  // update decreased the error success
	  model_ = new_model;
	  chi2 = new_chi2;
	  stop = norm_max(dX)<=eps;
	  mu *= max(1./3., min(1.-pow(2*rho-1,3), 2./3.));
	  nu = 2.;
	  if (verbose)
	  {
	    std::cout << "It. " << iter
		      << "\t trial " << n_trials
		      << "\t Success"
		      << "\t n_meas = " << n_meas
		      << "\t new_chi2 = " << new_chi2
		      << "\t mu = " << mu
		      << "\t nu = " << nu
		      << std::endl;
	  }
	}
	else
	{
	  //  fail
	  mu *= nu;
	  nu *=2;
	  n_trials++;
	  if (n_trials >= n_trial_max)
	    stop = true;
	  if (verbose)
	  {
	    std::cout << "It. " << iter
		      << "\t Trial " << n_trials
		      << "\t Failed"
		      << "\t n_meas = " << n_meas
		      << "\t new_chi2 = " << new_chi2
		      << "\t mu = " << mu
		      << "\t nu = " << nu
		      << std::endl;
	  }
	}
	
	finishIteration();
      }
      if (stop)
	break;
      
      finishIteration();
    }
  }
  
  template<int dim, typename T>
  void NLSSolver<dim, T>::setRobustCostFunction(ScaleEstimatorType scaleEstimator_, WeightFunctionType weightFunction_)
  {
    switch (scaleEstimator_)
    {
      case TDistScale:
	if (verbose)
	  std::cout << "using T-distribution scale estimator" << std::endl;
	scaleEstimator.reset(new TDistributionScaleEstimator());
	using_weight = true;
	break;
      case MADScale:
	if (verbose)
	  std::cout << "using MAD scale estimator" << std::endl;
	scaleEstimator.reset(new MADScaleEstimator());
	using_weight = true;
	break;
      case NormalScale:
	if (verbose)
	  std::cout << "using Normal scale estimator" << std::endl;
	scaleEstimator.reset(new NormalDistributionScaleEstimator());
	using_weight = true;
	break;
      default:
	if (verbose)
	  std::cout << "using unit scale estimator" << std::endl;
	scaleEstimator.reset(new UnitScaleEstimator());
	using_weight = false;
    }
    switch (weightFunction_)
    {
      case TDistWeight:
	if (verbose)
	  std::cout << "using T-distribution weight function" << std::endl;
	weightFunction.reset(new TDistributionWeightFunction());
	break;
      case TukeyWeight:
	if (verbose)
	  std::cout << "using Tukey weight function" << std::endl;
	weightFunction.reset(new TukeyWeightFunction());
	break;
      case HuberWeight:
	if (verbose)
	  std::cout << "using Huber weight function" << std::endl;
	weightFunction.reset(new TukeyWeightFunction());
	break;
      default:
	if (verbose)
	  std::cout << "using Unit weight function" << std::endl;
	weightFunction.reset(new UnitWeightFunction());
    }
  }
  
  template<int dim, typename T>
  void NLSSolver<dim, T>::setPrior(const T& prior_, const Matrix<double, dim, dim>& informationMatrix_)
  {
    have_prior = true;
    prior = prior_;
    InformationMatrix = informationMatrix_;
  }
  
  template<int dim, typename T>
  void NLSSolver<dim, T>::reset()
  {
    have_prior = false;
    chi2 = 1e10;
    mu = mu_init;
    nu = nu_init;
    n_meas = 0;
    n_iter = n_iter_init;
    iter = 0;
    stop = false;
  }
  
  template<int dim, typename T>
  const double& NLSSolver<dim, T>::getChi2() const 
  {
    return chi2;
  }
  
  template<int dim, typename T>
  const Matrix<double, dim, dim>& NLSSolver<dim, T>::getInformationMatrix() const
  {
    return H;
  }
} // end of namespace Vision

#endif