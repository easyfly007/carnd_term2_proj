// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include "Dense"

//
// Helper functions
//
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;


Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());
  
  //TODO: Implement the Global Kinematic Model, to return
  // the next state from inputs

  // NOTE: state is [x, y, psi, v]
  // NOTE: actuators is [delta, a]
  
  //Add your code below
  double x0 = state[0];
  double y0 = state[1];
  double psi0 = state[2];
  double v0 = state[3];
  
  double delta = actuators[0];
  double a = actuators[1];
  
  next_state[0] = x0 + v0 * cos(psi0) * dt;
  next_state[1] = y0 + v0 * sin(psi0) * dt;
  next_state[2] = psi0 + v0 / Lf * delta * dt;
  next_state[3] = v0 + a * dt;


  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  Eigen::VectorXd next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}