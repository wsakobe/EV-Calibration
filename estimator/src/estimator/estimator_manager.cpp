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

    double knot_distance = 0.1;
    trajectory_ = std::make_shared<Trajectory>(knot_distance);
    trajectory_manager_ = std::make_shared<TrajectoryManager>(trajectory_);
}

EstimatorManager::~EstimatorManager(){
    //shutdown every nodes
    circle_sub_.shutdown();
    corner_sub_.shutdown();

    //TODO shutdown pub nodes
}

void EstimatorManager::performEstimator(){
    static bool first_inside = true;
    if (first_inside){
        ROS_ERROR("Success initialized!");
        first_inside = false;
        setInitialState();
    }
    else{
        //calculate the pose

        //add into trajectory
        //trajectory_manager_->extendTrajectory(current_img_time + 0.04 * S_TO_NS);
    }

    // UpdateTrajectory (Per t second do once)
    //trajectory_manager_->UpdateTrajectory();

    // publish trajectory

}

void EstimatorManager::cornerArrayCallback(const corner_msgs::cornerArray& msg){
    if (!est_initializer.b_both_initialized){
        est_initializer.corner_buffer_.push_back(msg);
        init_lc.lock();
        if (!est_initializer.convInitialSucc()){
            if (est_initializer.judgeBufferStatus(CONV_CAM)){
                est_initializer.processConv();
            }
        }
        else{
            if (est_initializer.evInitialSucc()){
                ROS_INFO("Both camera initialized");
                est_initializer.estimateInitialExtrinsic();
            }        
        }
        init_lc.unlock();
    }
    else{
        ROS_INFO("Conv add trajectory");
        corner_buffer_.emplace_back(msg);
        performEstimator();
    }    
}

void EstimatorManager::circleArrayCallback(const circle_msgs::circleArray& msg){
    if (!est_initializer.b_both_initialized){
        est_initializer.circle_buffer_.push_back(msg);
        init_lc.lock();
        if (!est_initializer.evInitialSucc()){
            if (est_initializer.judgeBufferStatus(EV_CAM)){
                est_initializer.processEv();
            }
        }
        else{
            if (est_initializer.convInitialSucc()){
                ROS_INFO("Both camera initialized");
                est_initializer.estimateInitialExtrinsic();
            }        
        }
        init_lc.unlock();
    }
    else{
        ROS_INFO("Ev add trajectory");
        circle_buffer_.emplace_back(msg);
        performEstimator();
    }    
}

void EstimatorManager::setInitialState(){
    int64_t data_start_time = trajectory_->getDataStartTime();
    int64_t trajectory_start_time = est_initializer.corner_buffer_selected.begin()->timestamp.toSec() * S_TO_NS;
    trajectory_->setDataStartTime(trajectory_start_time);

    // skip all circle features before the first corner feature
    for (auto it = est_initializer.circle_buffer_.begin(); it != est_initializer.circle_buffer_.end();){
        if (it->header.stamp.toSec() * S_TO_NS < trajectory_start_time)
            it = est_initializer.circle_buffer_.erase(it);
        else
            break;
    }

    // skip all corner features before the first selected corner feature
    for (auto it = est_initializer.corner_buffer_.begin(); it != est_initializer.corner_buffer_.end();){
        if (it->timestamp.toSec() * S_TO_NS <= trajectory_start_time)
            it = est_initializer.corner_buffer_.erase(it);
        else
            break;
    }

    // set initial pose (conv camera)
    cv::Mat T;
    est_initializer.solveRelativePose(est_initializer.corner_buffer_.front(), est_initializer.convCameraMatrix, est_initializer.convDistCoeffs, T);
    Eigen::Matrix4d eigenT;
    cv::cv2eigen(T, eigenT);
    SE3d T_sop(eigenT);
    SO3d R0(T_sop.rotationMatrix());
    for (size_t i = 0; i <= trajectory_->numKnots(); i++) // only 4 control points at the very beginning
    {
      trajectory_->setKnotSO3(R0, i);
    }

    // add corner into trajectory
    for (auto &corner:est_initializer.corner_buffer_){
        est_initializer.solveRelativePose(corner, est_initializer.convCameraMatrix, est_initializer.convDistCoeffs, T);
        Eigen::Matrix4d eigenT;
        cv::cv2eigen(T, eigenT);
        SE3d se3(eigenT);
        int64_t current_img_time = corner.timestamp.toSec() * S_TO_NS;
        current_img_time -= trajectory_start_time;
        trajectory_manager_->extendTrajectory(current_img_time + 1000, se3);
        estimator->addConvFeatureAnalytic(corner);
    }

    // add circle into trajectory
    for (auto &circles:est_initializer.circle_buffer_){
        for (auto &circle:circles.circles){
            if (circle.timestamp.toSec() * S_TO_NS < trajectory_->getDataStartTime()){
                continue;
            }
            if (circle.timestamp.toSec() * S_TO_NS > trajectory_->maxTimeNs()){
                break;
            }
            estimator->addEventFeatureAnalytic(circle);
        }
    }

    //update trajectory once
    
}

};