#include "robot_motion_interface/interface.hpp"


namespace robot_motion_interface {
    
    
    void Interface::home(bool blocking) {
        set_joint_positions(home_joint_positions_, rp_->joint_names(), blocking);
    };
    

    
    
} 
    