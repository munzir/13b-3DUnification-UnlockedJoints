// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/30/19
// Brief: An implementation of inverse kinematics

#include <dart/dart.hpp>

#include <nlopt.hpp>

#include "ik.hpp"

// // compute speeds for joints based on ik algorithm
Eigen::VectorXd computeSpeeds(int mOptDim,
                              double (*optFunc)(const std::vector<double>& x,
                                                std::vector<double>& grad,
                                                void* my_func_data),
                              OptParams optParams, bool maxTimeSet,
                              Eigen::VectorXd mdqBodyRef) {
  nlopt::opt opt(nlopt::LD_SLSQP, mOptDim);
  double minf;
  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-3);
  if (maxTimeSet) opt.set_maxtime(0.01);
  std::vector<double> dqBodyRef_vec(mOptDim);
  Eigen::VectorXd::Map(&dqBodyRef_vec[0], mdqBodyRef.size()) = mdqBodyRef;
  try {
    // nlopt::result result = opt.optimize(dqBodyRef_vec, minf);
    opt.optimize(dqBodyRef_vec, minf);
  } catch (std::exception& e) {
    // std::cout << "nlopt failed: " << e.what() << std::endl;
  }

  for (int i = 0; i < mOptDim; i++) mdqBodyRef(i) = dqBodyRef_vec[i];

  Eigen::VectorXd initialSpeeds = mdqBodyRef;

  return initialSpeeds;
}
