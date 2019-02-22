#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "../include/Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

//calculate RMSE
VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &groundtruth);

//calculate Jacobians
MatrixXd CalculateJacobian(const VectorXd& x_state);

//Normalize the angle
float normalize_angle(float phi);

//normalize the bearing
void normalize_bearing(VectorXd& Z);

#endif
