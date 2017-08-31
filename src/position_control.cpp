#include "bebop_pos_ctrl.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pos_ctrl_bebop");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Bebop_Ctrl::bebop_pos_ctrl Bebop_Control(nh, pnh);
    
    // double set_x = 0.9;
    // double set_y = 1.5;
    // double set_z = -1.2;
    // double set_yaw = 0;

    // Bebop_Control.Control2Goal(set_x, set_y, set_z, set_yaw);
    // Bebop_Control.Control2Goal(0.5, 0.5, -1.0, set_yaw);

    return 0;
}

