#include <ros/ros.h>
#include "cliff_detector/cliff_detector.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "cliff_detector");
    
    CliffDetector cliff_detector_;

    ros::spin();

    return 0;
}