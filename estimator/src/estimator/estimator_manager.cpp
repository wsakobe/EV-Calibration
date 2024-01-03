#include "estimator_manager.h"

namespace estimator{

EstimatorManager::EstimatorManager(const YAML::Node &yaml_node, ros::NodeHandle& nh)
    : nh_(nh)
{
    circle_sub_ = nh_.subscribe("/circle_ev/circleArray", 10, &EstimatorManager::circleArrayCallback, this);
    corner_sub_ = nh_.subscribe("/corner_gs/cornerArray", 10, &EstimatorManager::cornerArrayCallback, this);
 
    //TODO setup trajectory
    est_initializer.initial_window_size = yaml_node["init_window_size"].as<int>();
    est_initializer.conv_cam_size = cv::Size(yaml_node["conv_cam_width"].as<int>(), yaml_node["conv_cam_height"].as<int>());
    est_initializer.ev_cam_size = cv::Size(yaml_node["ev_cam_width"].as<int>(), yaml_node["ev_cam_height"].as<int>());
    est_initializer.square_size = yaml_node["square_size"].as<float>();
}

void EstimatorManager::performEstimator(){
    static bool first_inside = true;
    if (first_inside){
        ROS_ERROR("Success initialized!");
        first_inside = false;
        //TODO initialize trajectory

    }
    else{
        //calculate the pose

        //add into trajectory

    }
    // Optimization

    // UpdateTrajectory

    // publish trajectory

}

void EstimatorManager::cornerArrayCallback(const corner_msgs::cornerArray& msg){
    //Initialization
    est_initializer.corner_buffer_.push_back(msg);
    if (!est_initializer.convInitialSucc()){
        if (est_initializer.judgeBufferStatus(CONV_CAM)){
            est_initializer.processConv();
            if (est_initializer.convInitialSucc()){
                //TODO store information
                ROS_INFO("Conv camera initialized");
            }
        }
    }
    else{
        if (est_initializer.evInitialSucc()){
            //TODO add new msg
            ROS_INFO("Conv add trajectory");
            performEstimator();
        }        
    }
}

void EstimatorManager::circleArrayCallback(const circle_msgs::circleArray& msg){
    //Initialization
    est_initializer.circle_buffer_.push_back(msg);
    if (!est_initializer.evInitialSucc()){
        if (est_initializer.judgeBufferStatus(EV_CAM)){
            est_initializer.processEv();
            if (est_initializer.evInitialSucc()){
                //TODO store information
                ROS_INFO("Ev camera initialized");
            }
        }
    }
    else{
        if (est_initializer.convInitialSucc()){
            //TODO add new msg
            ROS_INFO("Ev add trajectory");
            performEstimator();
        }        
    }
    
}

};