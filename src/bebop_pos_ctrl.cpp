#include "bebop_pos_ctrl/bebop_pos_ctrl.h"

namespace Bebop_Ctrl
{

bebop_pos_ctrl::bebop_pos_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh):nh_(nh)
{
    pnh.param("K_p_xy", K_p_xy, 0.1); 
    pnh.param("K_p_z", K_p_z, 0.1); 
    pnh.param("K_p_yaw", K_p_yaw, 1.0);

    pnh.param("K_i_xy", K_i_xy, 0.1); 
    pnh.param("K_i_z", K_i_z, 0.1); 
    pnh.param("K_i_yaw", K_i_yaw, 1.0);

    pnh.param("MaxV_xy", MaxV_xy, 0.3); 
    pnh.param("MaxV_z", MaxV_z, 0.2); 
    pnh.param("MaxV_yaw", MaxV_yaw, 10.0);

    pnh.param("Tolerance_hori_pos", Tolerance_hori_pos, 0.1); 
    pnh.param("Tolerance_vert_pos", Tolerance_vert_pos, 0.1); 
    pnh.param("Tolerance_yaw_rad", Tolerance_yaw_rad, 0.1);

    bebop_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);
    get_marker_pose = nh_.subscribe("/pos_uav",1,&bebop_pos_ctrl::BebopPoseCallback,this);
    x_last = pos_sub.position.x;
    y_last = pos_sub.position.y;
    z_last = pos_sub.position.z;
    bebop_pos_ctrl::Quat2Euler(pos_sub.orientation, euler_last); // rad
    yaw_last = euler_last.z;

    bebop_pos_ctrl::fillPatrolList();

    VecMovIt it = patrol_list_.begin();
    
    for(int i = 0; i < num_point; i++)
    {

    geometry_msgs::Twist temp_goal = patrol_list_.back();// remember the position arrived
    patrol_list_.pop_back();
    cout<<"current target position =:"<<temp_goal<<endl;
    bebop_pos_ctrl::Control2Goal(temp_goal.linear.x,temp_goal.linear.y, temp_goal.linear.z, temp_goal.angular.z);
    }


}

void bebop_pos_ctrl::fillPatrolList()
{
    geometry_msgs::Twist temp_goal;
    cv::FileStorage fs("/home/exbot/catkin_ws/src/bebop_pos_ctrl/config/list.yaml", cv::FileStorage::READ);
    if( !fs.isOpened() ) // if we have file with parameters, read them
    {
        std::cout<<"ERROR, cannot open list.yaml!"<<std::endl;
    }
    cv::FileNode list_n = fs["features"];
    cv::FileNodeIterator it = list_n.begin(), it_end = list_n.end();
    num_point = 0;
    for (; it != it_end; ++it)
    {
        temp_goal.linear.x   = (double)(*it)["x"];
        temp_goal.linear.y   = (double)(*it)["y"];
        temp_goal.linear.z   = (double)(*it)["z"];
        temp_goal.angular.x   = 0.0;
        temp_goal.angular.y   = 0.0;
        temp_goal.angular.z   = (double)(*it)["yaw"];

        std::cout << (double)(*it)["x"] << "  ";
        std::cout << (double)(*it)["y"] << "  ";
        std::cout << (double)(*it)["z"] << "  ";
        std::cout << (double)(*it)["yaw"] << std::endl;
        num_point ++;
        patrol_list_.push_back(temp_goal);
    }
    std::reverse(patrol_list_.begin(), patrol_list_.end());
    fs.release();
}

void bebop_pos_ctrl::Control2Goal(double set_x, double set_y, double set_z, double set_yaw)
{
    cout<<"entered Control2Goal!"<<endl;
    double get_x ,get_y,get_z, get_yaw; //yaw rad
    geometry_msgs::Vector3 get_euler;
    ros::Rate loopRate(20);
    while(ros::ok())
    {
        get_marker_pose = nh_.subscribe("/pos_uav",1,&bebop_pos_ctrl::BebopPoseCallback,this);
        usleep(30000);  // 10000ms can not receive correct data
        ros::spinOnce();

        get_x = pos_sub.position.x;
        get_y = pos_sub.position.y;
        get_z = pos_sub.position.z;
        cout<<"get_x: "<<get_x;
        cout<<"	    get_y: "<<get_y;
        cout<<"	    get_z: "<<get_z;

        bebop_pos_ctrl::Quat2Euler(pos_sub.orientation, get_euler); // rad
        get_yaw = get_euler.z;
        cout<<"    yaw(rad): "<<get_yaw<<endl;

        if(!(get_x == 0 && get_y == 0 && get_z == 0))
        {
            if(abs(get_x - set_x)<Tolerance_hori_pos && abs(get_y - set_y)<Tolerance_hori_pos && \
            abs(get_z-set_z)<Tolerance_vert_pos  && abs(get_yaw-set_yaw)< Tolerance_yaw_rad)
            {
                cout<<"Jump out the goal control loop!"<<endl;
                break;
            }
            else
            {
                double v_x =  K_p_xy*(cos(get_yaw)*(set_x-get_x) + sin(get_yaw)*(set_y-get_y));
                double v_y =  K_p_xy*(-sin(get_yaw)*(set_x-get_x) + cos(get_yaw)*(set_y-get_y));
                double v_z =  K_p_z*(set_z - get_z);
                double v_yaw = K_p_yaw*(set_yaw - get_yaw);  
                cout<<"vel_x =:"<<v_x<<"    vel_y =:"<<v_y<<endl; 

                bebop_pos_ctrl::Limitator(v_x, v_y, v_z, v_yaw); 
                     
                
                /*
                linear.x (+)  fly forward
                         (-)  fly backward
                linear.y (+)  fly left
                         (-)  fly right
                linear.z (+)  fly up
                         (+)  fly down
                angular.z(+)  rotate counter clockwise    
                         (-)  rotate clockwise        
                */
                cmd_vel_pub.linear.x = v_x;
                cmd_vel_pub.linear.y = -v_y; 
                cmd_vel_pub.linear.z = -v_z;
                cmd_vel_pub.angular.x = 0;
                cmd_vel_pub.angular.y = 0;
                cmd_vel_pub.angular.z = -v_yaw;
                bebop_cmd_vel.publish(cmd_vel_pub);

            }
        }
        loopRate.sleep();
    }
    cout<<"leave Control2Goal!"<<endl;

}

void bebop_pos_ctrl::BebopPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    pos_sub = *msg;
}

void bebop_pos_ctrl::Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler)
{
    double q0 = quat.w;
    double q1 = quat.x;
    double q2 = quat.y;
    double q3 = quat.z;

    double t0 = -2.0 * (q2 * q2 + q3 * q3) + 1.0;
    double t1 = +2.0 * (q1 * q2 + q0 * q3);
    double t2 = -2.0 * (q1 * q3 - q0 * q2);
    double t3 = +2.0 * (q2 * q3 + q0 * q1);
    double t4 = -2.0 * (q1 * q1 + q2 * q2) + 1.0;

    //t2 = t2 > 1.0 ? 1.0 : t2;
    //t2 = t2 < -1.0 ? -1.0 : t2;

    // cout<<"pitch =  :"<< asin(t2)<<endl;
	// cout<<"roll  =  :"<< atan2(t3, t4)<<endl;
 	// cout<<"yaw   =  :"<< atan2(t1, t0)<<endl;

 	euler.x = atan2(t3, t4);
 	euler.y = asin(t2);
 	euler.z = atan2(t1, t0);
}

void bebop_pos_ctrl::Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat)
{
    double fi  = euler.x / 2;
    double theta = euler.y / 2;
    double psi   = euler.z / 2;
    
    quat.w = cos(fi)*cos(theta)*cos(psi) + sin(fi)*sin(theta)*sin(psi);  
    quat.x = sin(fi)*cos(theta)*cos(psi) - cos(fi)*sin(theta)*sin(psi);
    quat.y = cos(fi)*sin(theta)*cos(psi) + sin(fi)*cos(theta)*sin(psi);
    quat.z = cos(fi)*cos(theta)*sin(psi) - sin(fi)*sin(theta)*cos(psi);
}
void bebop_pos_ctrl::Limitator(double& vx, double& vy, double& vz, double& yawrate)
{
    if(vx < (- MaxV_xy))
        vx = - MaxV_xy;
    else if (vx > MaxV_xy)
        vx = MaxV_xy;

    if(vy < (- MaxV_xy))
        vy = - MaxV_xy;
    else if (vy > MaxV_xy)
        vy = MaxV_xy; 

    if(vz < (- MaxV_z))
        vz = - MaxV_z;
    else if(vz > MaxV_z)
        vz = MaxV_z;

    if(yawrate < (- MaxV_yaw))
        yawrate = - MaxV_yaw;
    else if(yawrate > MaxV_yaw)
        yawrate = MaxV_yaw;     
}

}