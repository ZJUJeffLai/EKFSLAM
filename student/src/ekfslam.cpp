#include "ekfslam.h"
#include "../include/common.h"
#include "../include/Eigen/Dense"
/****** TODO *********/
    // Overloaded Constructor
    // Inputs:
    // landmark_size - number of landmarks on the map
    // robot_pose_size - number of state variables to track on the robot
    // motion_noise - amount of noise to add due to motion
EKFSLAM::EKFSLAM(unsigned int landmark_size,unsigned int robot_pose_size = 3,float _motion_noise = 0.1)
{
    //full state vector(initialization)
    mu = VectorXd::Zero(2*landmark_size + robot_pose_size,1);
    robotSigma = MatrixXd::Zero(robot_pose_size,robot_pose_size);
    robMapSigma = MatrixXd::Zero(robot_pose_size,2*landmark_size);
    mapSigma = INF*MatrixXd::Identity(2*landmark_size,2*landmark_size);
    Sigma = MatrixXd::Zero(2*landmark_size+robot_pose_size,2*landmark_size+robot_pose_size);

    Sigma.topLeftCorner(robot_pose_size,robot_pose_size) = robotSigma;
    Sigma.topRightCorner(robot_pose_size,2*landmark_size) = robMapSigma;
    Sigma.bottomLeftCorner(2*landmark_size,robot_pose_size) = robMapSigma.transpose();
    Sigma.bottomRightCorner(2*landmark_size,2*landmark_size) = mapSigma;

    float motion_noise = _motion_noise;
    R = MatrixXd:Zero(2*landmark_size+robot_pose_size,2*landmark_size+robot_pose_size);
    R.topLeftCorner(3,3)<<motion_noise,0,0,
        0,motion_noise,0,
        0,0,motion_noise;
    


    observedLandmarks.resize(landmark_size);
    fill(observedLandmarks.begin(),observedLandmarks.end(),false);
}

/****** TODO *********/
    // Description: Prediction step for the EKF based off an odometry model
    // Inputs:
    // motion - struct with the control input for one time step

void EKFSLAM::Prediction(const OdoReading& motion)
{
    double angle = mu(2);
    double r1 = motion.r1;
    double t = motion.t;
    double r2 = motion.r2;

    MatrixXd Gt = MatrixXd(3,3);
    
    float value_cos = cos(angle+r1);
    float value_sin = sin(angle+r1);
    Gt<<1,0,-t*value_sin,
        0,1,t*value_cos,
        0,0,0;
    
    mu(0) = mu(0) + t*value_cos;
    mu(1) = mu(1) + t*value_sin;
    mu(2) = mu(2) + r1 + r2;

    int sigma_cols = Sigma.cols();
    Sigma.topLeftCorner(3,3) = Gt*Sigma.topLeftCorner(3,3)*Gt.transpose();
    Sigma.topRightCorner(3,sigma_cols-3) = Gt*Sigma.topRightCorner(3,sigma_cols-3);
    Sigma.bottomLeftCorner(sigma_cols-3,3) = Sigma.topRightCorner(3,sigma_cols-3).transpose();

    Sigma = Sigma + R;

    
}


/****** TODO *********/
    // Description: Prediction for the EKF based off a velocity model
    // Inputs:
    // motion - struct with velocity-based control input
void EKFSLAM::Prediction(const VelReading& motion)
{

}

/****** TODO *********/
    // Description: Prediction for the EKF based off two sensor inputs
    // Inputs:
    // odom - control vector based off an odometry model
    // vel - control vector based off a velocity model
void EKFSLAM::Prediction(const OdoReading& odom, const VelReading& vel)
{

}


/****** TODO *********/
    // Description: Correction step for EKF
    // Inputs:
    // observation - vector containing all observed landmarks from a laser scanner
void EKFSLAM::Correction(const vector<LaserReading>& observation)
{

}