#include "estimator_manager.h"

namespace estimator{

EstimatorManager::EstimatorManager(const YAML::Node &yaml_node, ros::NodeHandle& nh)
    : nh_(nh)
{
    circle_sub_ = nh_.subscribe("/circle_ev/circleArray", 10, &EstimatorManager::circleArrayCallback, this);
    corner_sub_ = nh_.subscribe("/corner_gs/cornerArray", 10, &EstimatorManager::cornerArrayCallback, this);

    //TODO setup trajectory

    //Initialization
    
    //Optimization
    
}

void EstimatorManager::cornerArrayCallback(const corner_msgs::cornerArray& msg){
    for (auto &m : msg){
        corner_msgs::cornerArray temp_corner_;
        temp_corner.
    }
}

void EstimatorManager::circleArrayCallback(const circle_msgs::circleArray& msg){

}

};