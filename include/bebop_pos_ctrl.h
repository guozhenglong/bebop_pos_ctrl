
#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
typedef std::vector<geometry_msgs::Twist>::iterator VecMovIt;

using namespace std;

namespace Bebop_Ctrl{
class bebop_pos_ctrl{
    public:
        bebop_pos_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh);

        void fillPatrolList();
        void Control2Goal(double set_x, double set_y, double set_z, double set_yaw);//-+
        void BebopPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

        void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler);
        void Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat);
        void Limitator(double& vx, double& vy, double& vz, double& yawrate);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber get_marker_pose;
        ros::Publisher  bebop_cmd_vel;
        geometry_msgs::Pose pos_sub;
        geometry_msgs::Twist cmd_vel_pub;
        std::vector<geometry_msgs::Twist> patrol_list_;
        int num_point;

        double  K_p_xy,K_p_z ,K_p_yaw,K_i_xy,K_i_z ,K_i_yaw;
        double  MaxV_xy, MaxV_z , MaxV_yaw,Tolerance_hori_pos,Tolerance_vert_pos,Tolerance_yaw_rad;
        double  x_last, y_last, z_last, yaw_last;
        geometry_msgs::Vector3 euler_last;


};
}






