#include "circle_detector.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_detector");
    ros::NodeHandle nh;
    
    circle_detector::CircleDetector cd;
    circle_detector::EventProcessor ep(nh, cd);

    ros::spin();

    return 0;
}
