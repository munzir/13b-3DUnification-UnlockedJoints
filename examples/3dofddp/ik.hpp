// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/30/19
// Brief: An implementation of inverse kinematics

#ifndef IK_HPP
#define IK_HPP

#include <dart/dart.hpp>
#include <nlopt.hpp>

struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

// Function Prototypes
// // compute speeds for joints based on ik algorithm
Eigen::VectorXd computeSpeeds(int mOptDim,
                              double (*optFunc)(const std::vector<double>& x,
                                                std::vector<double>& grad,
                                                void* my_func_data),
                              OptParams optParams, bool maxTimeSet,
                              Eigen::VectorXd mdqBodyRef);

#endif  // IK_HPP
