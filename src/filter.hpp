// Author: Munzir Zafar
// Ported by: Akash Patel (apatel435)
// Date: 3/31/19
// Brief: Filter Class

#include <Eigen/Eigen>
#include <boost/circular_buffer.hpp>

class filter {
 public:
  filter(const int dim, const int n) {
    samples.set_capacity(n);
    total = Eigen::VectorXd::Zero(dim, 1);
  }
  void AddSample(Eigen::VectorXd v) {
    if (samples.full()) {
      total -= samples.front();
    }
    samples.push_back(v);
    total += v;
    average = total / samples.size();
  }

  boost::circular_buffer<Eigen::VectorXd> samples;
  Eigen::VectorXd total;
  Eigen::VectorXd average;
};
