#include<gp_reg/GpRegNode.h>

GpRegNode::GpRegNode() {

    n_.getParam("/gp_reg_node/sub_topic",sub_topic_);
    n_.param("/gp_reg_node/draw", draw_, true);
    sub_ = n_.subscribe(sub_topic_, 10, &GpRegNode::subCallback, this);
    ros::spin();

};

void GpRegNode::gpRegression(){

};
void GpRegNode::drawRegression(){
    plt::plot({1,3,2,4});
    plt::show();
    plt::save("fig.png");
};

void GpRegNode::subCallback (const sensor_msgs::LaserScanConstPtr& scan){
    this->raw_scan_ = *scan;
    gpRegression();

    if (draw_)
       drawRegression();
};

