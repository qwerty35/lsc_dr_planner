//
// Created by jungwon on 22. 2. 8..
//

#ifndef DYNAMIC_PLANNER_LINEARKALMANFILTER_H
#define DYNAMIC_PLANNER_LINEARKALMANFILTER_H



#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <string>

#define PUBLISH_RAW_TWIST false

#define Row 6
#define Col 3

using namespace Eigen;
typedef Matrix<double, Row, Row> FMatrix;
typedef Matrix<double, Row, Col> GMatrix;
typedef Matrix<double, Row, Row> QMatrix;
typedef Matrix<double, Row, Row> PMatrix;
typedef Matrix<double, Col, Col> RMatrix;
typedef Matrix<double, Col, Row> HMatrix;
typedef Matrix<double, Row, 1> NVector;
typedef Matrix<double, Col, 1> MVector;

class LinearKalmanFilter {
public:
    LinearKalmanFilter();
    nav_msgs::Odometry pose_cb(const geometry_msgs::PoseStamped& msg);

private:
    FMatrix F;
    GMatrix G;
    QMatrix Q;
//    RMatrix R;
    HMatrix H;

    NVector x_old;
    NVector x_predict;
    NVector x_estimate;
    PMatrix P_old;
    PMatrix P_predict;
    PMatrix P_estimate;

    MVector sigma_Q;
    MVector sigma_R;

    geometry_msgs::PoseStamped pose_old, pose_new;
    geometry_msgs::TwistStamped twist, twist_raw;
    nav_msgs::Odometry odom;

    ros::WallTime t_old, t_new;
    bool initialized = false;

    void predict(const double &dt);
    static FMatrix computeF(const double &dt);
    static GMatrix computeG(const double &dt);
    static QMatrix computeQ(const GMatrix &G, const MVector &sigma_Q);
    void update(const double &dt, const geometry_msgs::PoseStamped& msg);
    RMatrix computeR();
};

#endif //DYNAMIC_PLANNER_LINEARKALMANFILTER_H
