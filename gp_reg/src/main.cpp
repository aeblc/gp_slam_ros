#include<iostream>
#include<Eigen/Dense>
#include<gp_reg/GpRegNode.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "gp_reg_node");
    //ros::NodeHandle pn ("~");
    Eigen::Vector3d v(0,0,0);
    std::cout << "gp_reg" << std::endl;
    GpRegNode* k = new GpRegNode();
    return 0;
}
