#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "../include/Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Calculate RMSE
VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &groundtruth);

// Calculate Jacobians
MatrixXd CalculateJacobian(const VectorXd& x_state);

// Normalize the angle
float normalize_angle(float phi);

// Normalize the bearing
void normalize_bearing(VectorXd& Z);

#endif
