//After lights have been triggered takes a capture

#include "realsenseCamera.h"

RealsenseCamera::RealsenseCamera():Node("realsense_camera"){
    
}



int RealsenseCamera::capturePointCloud(){
    capturedPointCloud_ = currentPointCloud_;
    return 1;
}

sensor_msgs::msg::PointCloud2 RealsenseCamera::getCurrentPointCloud(){
    return currentPointCloud_;
}