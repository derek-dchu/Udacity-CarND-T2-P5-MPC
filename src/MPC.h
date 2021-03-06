#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "helper_functions.h"


using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
  Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, int latency);
};

#endif /* MPC_H */
