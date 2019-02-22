#include <iostream>
#include "tools.h"
#define M_PI 3.14159265358979323846

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


VectorXd CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& groundtruth)
{
    VectorXd rmse(4);
    rmse<<0,0,0,0;

    if(estimations.size()!=groundtruth.size() || estimations.size()==0) {
        std::cout<<"Invaild!"<<std::endl;
        return rmse;
    }

    for(unsigned int i = 0; i < estimations.size(); i++) {
        VectorXd residual = estimations[i] - groundtruth[i];
        residual = residual.array()*residual.array();
        rmse = rmse + residual;
    }
    // Mean
    rmse = rmse/estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd CalculateJacobian(const VectorXd& x_state)
{
    MatrixXd Hj(3,4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float cal1 = px*px + py*py;
    float cal2 = sqrt(cal1);
    float cal3 = cal1*cal2;

    //Compute Jacobian Matrix
    Hj<<(px/cal2),(py/cal2),0,0,
        -(py/cal1),(px/cal1),0,0,
        py*(vx*py-vy*px)/cal3,px*(px*vy-py*vx)/cal3,px/cal2,py/cal2;
    return Hj;
}

float normalize_angle(float phi)
{
    while(phi>M_PI) {
        phi = phi - 2*M_PI;
    }
    while(phi<-M_PI) {
        phi = phi + 2*M_PI;
    }
    return phi;
}

void normalize_bearing(VectorXd& Z)
{
    for(int i = 1; i < Z.size(); i = i+2) {
        Z(i) = normalize_angle(Z(i));
    }
}
