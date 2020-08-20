#include<gp_reg/GpRegNode.h>

GpRegNode::GpRegNode() {

    n_.getParam("/gp_reg_node/sub_topic",sub_topic_);
    n_.getParam("/gp_reg_node/draw", draw_);
    n_.getParam("/gp_reg_node/downsampled_data_size", downsampled_data_size_);
    n_.getParam("/gp_reg_node/estimation_data_size", estimation_data_size_);

    n_.getParam("/gp_reg_node/length_scale", length_scale_);
    n_.getParam("/gp_reg_node/signal_variance", signal_variance_);
    n_.getParam("/gp_reg_node/observation_noise_", observation_noise_);



    ROS_INFO("Estimating LaserScan data with %d points !!", downsampled_data_size_);

    sub_ = n_.subscribe(sub_topic_, 10, &GpRegNode::subCallback, this);


    // spin is in main!!
};


double kernelFunction(double x1, double x2, double lambda, double sigma){

    return sigma *  sigma * std::exp( -2 * std::sin((x1 - x2) / 2) * std::sin((x1 - x2) / 2) / (lambda * lambda) ) ;
}

void GpRegNode::gpRegression(){

    unsigned int  laserdata_size = raw_scan_.ranges.size();

    r_theta_.resize(laserdata_size, 2); // n rows , 2 cols
    
    for(int n = 0; n < laserdata_size ; ++n){
    
        if(raw_scan_.ranges[n] <= raw_scan_.range_max){
            r_theta_(n,0) = raw_scan_.ranges[n];                                 // ranges
            r_theta_(n,1) = raw_scan_.angle_min + raw_scan_.angle_increment * n; // angles
        }
        
        else{
            r_theta_(n,0) = raw_scan_.range_max;
            r_theta_(n,1) = raw_scan_.angle_min + raw_scan_.angle_increment * n;
        }

    }

    // downsample
    downsampled_data_.resize(downsampled_data_size_, 2);
    unsigned int jump_interval = laserdata_size / downsampled_data_size_;

    for(int n = 0 ; n < downsampled_data_size_ ; ++n){
    
       downsampled_data_(n,0) = r_theta_(n * jump_interval, 0); //ranges
       downsampled_data_(n,1) = r_theta_(n * jump_interval, 1); //angles

    }

    // estimation interval (-pi, pi)
    estimation_interval_ =  Eigen::VectorXd::LinSpaced(estimation_data_size_, -1 * M_PI, M_PI );

    // calculate covariance matrices
    Eigen::MatrixXd K(downsampled_data_size_, downsampled_data_size_);
    Eigen::MatrixXd K_star(estimation_data_size_, downsampled_data_size_);
    Eigen::MatrixXd K_star_star(estimation_data_size_, estimation_data_size_);

    for(int i = 0; i < downsampled_data_size_; ++i)
        for(int j = 0; j < downsampled_data_size_; ++j )
            K(i,j) = kernelFunction(downsampled_data_(i,1), downsampled_data_(j,1), length_scale_ , signal_variance_);

    for(int i = 0; i < estimation_data_size_; ++i)
        for(int j = 0; j < downsampled_data_size_; ++j )
            K_star(i,j) = kernelFunction(estimation_interval_(i), downsampled_data_(j, 1), length_scale_, signal_variance_);

     for(int i = 0; i < estimation_data_size_; ++i)
        for(int j = 0; j < estimation_data_size_; ++j )
            K_star_star(i,j) = kernelFunction(estimation_interval_(i), estimation_interval_(j), length_scale_, signal_variance_);
   

     // observation errors
     K = K + Eigen::MatrixXd::Identity(downsampled_data_size_, downsampled_data_size_) * observation_noise_ * observation_noise_;

     // calculate posterior
     posterior_mean_ = K_star * K.inverse() * downsampled_data_.col(0) ;
     posterior_covariance_ = K_star_star - K_star * K.inverse() * K_star.transpose() ;

        
};
void GpRegNode::drawRegression(){
    
    plt::ion();
    Eigen::MatrixXd a1 = r_theta_.col(0);
    Eigen::MatrixXd b1 = r_theta_.col(1);
    std::vector<double> veca1(a1.data(), a1.data() + a1.size());
    std::vector<double> vecb1(b1.data(), b1.data() + b1.size());
    plt::plot(vecb1, veca1, {{"color", "red"} ,{"label", "raw_data"}});

    Eigen::MatrixXd a2 = posterior_mean_;
    Eigen::MatrixXd b2 = estimation_interval_; 
    std::vector<double> veca2(a2.data(), a2.data() + a2.size());
    std::vector<double> vecb2(b2.data(), b2.data() + b2.size());
    plt::plot(vecb2, veca2, {{"color", "blue"}, {"linestyle", "--"}, {"label", "estimation"}});
    plt::legend();

    plt::grid(true);
    plt::xlabel("Angles (rad)");
    plt::ylabel("Ranges (m)");
    plt::show();
    plt::pause(0.1);
    plt::clf();
};

void GpRegNode::subCallback (const sensor_msgs::LaserScanConstPtr& scan){
    this->raw_scan_ = *scan;
    gpRegression();

    if (draw_)
       drawRegression();
};

