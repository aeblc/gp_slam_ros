#include<gp_reg/GpRegNode.h>

GpRegNode::GpRegNode() {

    n_.getParam("/gp_reg_node/sub_topic",sub_topic_);
    n_.param("/gp_reg_node/draw", draw_, true);
    sub_ = n_.subscribe(sub_topic_, 10, &GpRegNode::subCallback, this);
    r_theta_.resize(2,2);

    ros::spin();

};

void GpRegNode::gpRegression(){

    r_theta_.resize(2, raw_scan_.ranges.size());
    
    for(int n = 0; n < raw_scan_.ranges.size(); ++n){
    
        r_theta_(0,n) = raw_scan_.ranges[n];
        r_theta_(1,n) = raw_scan_.angle_min + raw_scan_.angle_increment * n;

    }

};
void GpRegNode::drawRegression(){
    
    plt::ion();
    Eigen::MatrixXd a = r_theta_.row(0);
    Eigen::MatrixXd b = r_theta_.row(1);
    std::vector<double> veca(a.data(), a.data() + a.size());
    std::vector<double> vecb(b.data(), b.data() + b.size());
    plt::plot(vecb,veca);
    plt::grid(true);
    plt::show();
    plt::pause(0.5);
    plt::clf();
    //plt::save("fig.png");
};

void GpRegNode::subCallback (const sensor_msgs::LaserScanConstPtr& scan){
    this->raw_scan_ = *scan;
    gpRegression();

    if (draw_)
       drawRegression();
};

