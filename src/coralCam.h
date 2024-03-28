#include "button.h"
#include "gui.h"
#include "industrialCamera.h"
#include "lights.h"
#include "realsenseCamera.h"


class CoralCam : public rclcpp::Node{
    public:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constructors and Destructors                                                                          //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        
            CoralCam();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Public Methods                                                                                        //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        
    private:

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Private Methods                                                                                       //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node, Publishers and Subscribers                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constants                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Variables                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        Button button_;
        Gui gui_;
        IndustrialCamera industrialCamera_;
        Lights lights_;
        RealsenseCamera realsenseCamera_;

};