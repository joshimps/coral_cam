#include "button.h"
#include "gui.h"
#include "industrialCamera.h"
#include "lights.h"
#include "realsenseCamera.h"


class CoralCam:public rclcpp::Node{
    public:
        CoralCam();
    private:
        Button button_;
        RealsenseCamera realsenseCamera_;

};