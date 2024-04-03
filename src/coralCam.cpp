#include "coralCam.h"

CoralCam::CoralCam(int buttonPinNumber,int ledNumber):Node("coral_cam_node"){
    button_ = new Button(buttonPinNumber);
    gui_ = new Gui();
    industrialCamera_ = new IndustrialCamera();
    realsenseCamera_ = new RealsenseCamera();
}