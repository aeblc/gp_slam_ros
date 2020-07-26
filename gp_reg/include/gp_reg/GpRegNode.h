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
    Eigen::Matrix2Xd r_theta_;

    ros::NodeHandle n_;
    ros::Subscriber sub_;
    std::string sub_topic_;
    bool draw_;

};


#endif
