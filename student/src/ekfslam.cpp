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
        0,0,motion_noise/10;
    


    observedLandmarks.resize(landmark_size);
    fill(observedLandmarks.begin(),observedLandmarks.end(),false);
}

/****** TODO *********/
    // Description: Prediction step for the EKF based off an odometry model
    // Inputs:
    // motion - struct with the control input for one time step

void EKFSLAM::Prediction(const OdoReading& motion)
{

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