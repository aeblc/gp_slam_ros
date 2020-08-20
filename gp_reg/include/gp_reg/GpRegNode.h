#ifndef GP_REG_NODE_H
#define GP_REG_NODE_H

#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<Eigen/Dense>
#include<matplotlib-cpp/matplotlibcpp.h>

namespace plt =  matplotlibcpp;

class GpRegNode
{
public:
    GpRegNode();
    void gpRegression();
    void drawRegression();
    void subCallback(const sensor_msgs::LaserScanConstPtr& scan);
private:
    sensor_msgs::LaserScan raw_scan_;
    Eigen::MatrixXd r_theta_;
    Eigen::MatrixXd downsampled_data_;
    Eigen::VectorXd estimation_interval_;
    Eigen::VectorXd posterior_mean_;
    Eigen::MatrixXd posterior_covariance_;
    double t;

    ros::NodeHandle n_;
    ros::Subscriber sub_;
    std::string sub_topic_;
    bool draw_;
    int downsampled_data_size_;
    double observation_noise_;

    // Hyper-parameters
    
    double length_scale_;
    double signal_variance_;
};


#endif
