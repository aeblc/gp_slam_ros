#include<Eigen/Dense>
#include<gp_reg/GpRegNode.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "gp_reg_node");

    GpRegNode node;

    ros::spin();

    return 0;
}
