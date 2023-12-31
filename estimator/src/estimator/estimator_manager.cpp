#include "estimator_manager.h"

namespace estimator{

EstimatorManager::EstimatorManager(const YAML::Node &yaml_node, ros::NodeHandle& nh)
    : nh_(nh)
{
    circle_sub_ = nh_.subscribe("/circle_ev/circleArray", 10, &EstimatorManager::circleArrayCallback, this);
    corner_sub_ = nh_.subscribe("/corner_gs/cornerArray", 10, &EstimatorManager::cornerArrayCallback, this);

    //TODO setup trajectory
    est_initializer.initial_window_size = yaml_node["init_window_size"].as<int>();
    

    //Initialization
    if (!est_initializer.initialSucc() && est_initializer.judgeBufferStatus(est_initializer.initial_window_size)){
        est_initializer.process();
        if (est_initializer.initialSucc()){
            //TODO set intial trajectory
        }
    }
    else{
        //TODO add new features into trajectory
    }
    
    //Optimization
    if (est_initializer.initialSucc()){
        // UpdateTrajectory
        // publish trajectory
    }
}

void EstimatorManager::cornerArrayCallback(const corner_msgs::cornerArray& msg){
    est_initializer.corner_buffer_.push_back(msg);
}

void EstimatorManager::circleArrayCallback(const circle_msgs::circleArray& msg){
    est_initializer.circle_buffer_.push_back(msg);
}

};