
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

 //ROS Images
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>


//Aruco Messages
#include "aruco_eye_msgs/PointInImage.h"
#include "aruco_eye_msgs/Marker.h"
#include "aruco_eye_msgs/MarkerList.h"
using namespace std;

#define dx 0.3
#define dy 0.3
geometry_msgs::PoseStamped  pos_pub;




// for kalman filter, x -> pos, z -> measurement
Eigen::VectorXd x_e_;
Eigen::VectorXd x_e;
Eigen::Vector3d z_m;
Eigen::MatrixXd F(6, 6);
Eigen::MatrixXd Tao(6, 6);
Eigen::MatrixXd H(3, 6);
Eigen::MatrixXd P_(6, 6);
Eigen::MatrixXd P(6, 6);
Eigen::MatrixXd Q(6, 6);
Eigen::Matrix3d R(3, 3);
Eigen::MatrixXd K(6, 6);
Eigen::MatrixXd I_6(6, 6);
Eigen::Matrix3d I_3(3, 3);
geometry_msgs::PoseStamped  pos_kf_pub;
geometry_msgs::Point pos_kf;






void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler);
void MarkerPoseCallback(const aruco_eye_msgs::MarkerList& msg);
void Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat);
void Quat2Rota(geometry_msgs::Quaternion &quat, Eigen::Matrix3d& rot);
void Quat_ComPen_Cam2Tag(geometry_msgs::Quaternion& q);


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "get_pos_uav");
    ros::NodeHandle nh;
    ros::Rate loopRate(30);

    pos_kf.x = 0;
    pos_kf.y = 0;
    pos_kf.z = 0;
    x_e<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    F<< 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;   
    Tao<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    H<< 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    I_6<< 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    I_3<< 1.0, 0.0, 0.0, 
          0.0, 1.0, 0.0, 
          0.0, 0.0, 1.0; 

    P = 100 * I_6;
    Q = 100 * I_6;
    R = 100 * I_3;

    ros::Publisher pos_uav = nh.advertise<geometry_msgs::PoseStamped>("/pos_uav",1);
    ros::Publisher pos_uav_kf = nh.advertise<geometry_msgs::PoseStamped>("/pos_uav_kf",1);
    ros::Subscriber get_marker_pose = nh.subscribe("/aruco_eye/aruco_observation",1,&MarkerPoseCallback);

    while(ros::ok())
    {
        pos_uav.publish(pos_pub);
        pos_uav_kf.publish(pos_kf_pub);
    	ros::spinOnce();
    	loopRate.sleep();
    }
    return 0;
}

void MarkerPoseCallback(const aruco_eye_msgs::MarkerList& msg)
{
    int count_markers;
    geometry_msgs::Point pos_marker;
    geometry_msgs::Point pos_ave;
    geometry_msgs::Quaternion quat_marker;
    geometry_msgs::Quaternion quat_ave;
    geometry_msgs::Vector3 euler;
    geometry_msgs::Vector3 rpy;
    
    Eigen::Matrix3d rot(3, 3);
    Eigen::Vector3d pos_in_body;
    Eigen::Vector3d pos_in_tag;
    double roll, pitch, yaw;
    int ID;

    
    pos_ave.x = 0;
    pos_ave.y = 0;
    pos_ave.z = 0;
    quat_ave.x = 0;
    quat_ave.y = 0;
    quat_ave.z = 0;
    quat_ave.w = 0; 

    if(!(msg.markers.empty()))
    {
        count_markers = msg.markers.size();
        for(int i=0;i<count_markers;i++)
        {
            quat_marker =  msg.markers[i].pose.pose.orientation;
            // The camera coordinate is different from the tag coordiante ,
            //so the quaternion includes this transform and we need to compensate it. 
            Quat_ComPen_Cam2Tag(quat_marker);
            Quat2Euler(quat_marker,rpy);
            roll  += rpy.x;
            pitch += rpy.y;
            yaw   += rpy.z;
        }
        roll  /= count_markers;
        pitch /= count_markers;
        yaw   /= count_markers;
        euler.x = roll;
        euler.y = pitch;
        euler.z = - yaw; // the image is opposite to the real scene. The left in image is the right in bebop, the right in image is the left in bebop.
        Euler2Quat(euler, quat_ave);
        cout<<"Roll =:"<<euler.x<<"     Pitch =:"<<euler.y<<"     Yaw =:"<<euler.z<<endl;

        for(int i=0;i<count_markers;i++)
        {
            //the angle of camera pitch limit is 83.0 deg.
            pos_marker = msg.markers[i].pose.pose.position;
            ID = msg.markers[i].id;
            //marker.x point to the y axis of body coordinate; marker.y point to the -x axis
            /*
            tags coordinate is opposite to the body coordinate, when the front of bebop is point north
            Tags coordinate:
            x
            ^    
            |   0   1   2   3   4   5   6   7   8   9
            |  10  11  12  13  14  15  16  17  18  19
            |  .
            |  .
            |  .
            |  90  91  92  93  94  95  96  97  98  99 
            O-------------------------------------------> y 
            */
            double limit_pitch_cam = (90-83)/180*3.14159;
            pos_in_body(0) = (- pos_marker.y)*cos(limit_pitch_cam) + (pos_marker.z)*sin(limit_pitch_cam);
            pos_in_body(1) = pos_marker.x;
            pos_in_body(2) = -(- pos_marker.y)*sin(limit_pitch_cam) + (pos_marker.z)*cos(limit_pitch_cam);
        
            Quat2Rota(quat_ave,rot);
            pos_in_tag = (rot.transpose())*(-pos_in_body);
            pos_ave.x += ( pos_in_tag(0) + (9-(int)(ID/10))*dx); 
            pos_ave.y += ( pos_in_tag(1) + (ID%10)*dy);
            pos_ave.z += (pos_in_tag(2)); 
        }
    	pos_ave.x /= count_markers;
        pos_ave.y /= count_markers;
        pos_ave.z /= count_markers;
        cout<<"X =:"<<pos_ave.x<<"     Y =:"<<pos_ave.y<<"     Z =:"<<pos_ave.z<<endl;


        // Kalman Filter
        z_m(0) = pos_ave.x;
        z_m(1) = pos_ave.y;
        z_m(2) = pos_ave.z;
        x_e_ = F * x_e ;
        P_   = F * P * F.transpose() + Tao * Q * Tao.transpose();
        K    = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
        x_e  = x_e_ + K * (z_m - H * x_e_);
        P    = (I_6 - K * H) * P_;

        pos_kf.x = x_e(0);
        pos_kf.y = x_e(1);
        pos_kf.z = x_e(2);

        pos_pub.header.frame_id = "bebop_pos";
        pos_pub.header.stamp = ros::Time::now();
        pos_pub.pose.position = pos_ave;
        pos_pub.pose.orientation = quat_ave;

        pos_kf_pub.header.frame_id = "bebop_pos_kf";
        pos_kf_pub.header.stamp = pos_pub.header.stamp;
        pos_kf_pub.pose.position = pos_kf;
        pos_kf_pub.pose.orientation = quat_ave;
    }
}

void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler)
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

    // t2 = t2 > 1.0 ? 1.0 : t2;
    // t2 = t2 < -1.0 ? -1.0 : t2;

 	euler.x = atan2(t3, t4);
 	euler.y = asin(t2);
 	euler.z = atan2(t1, t0);
}

void Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat)
{
    double fi  = euler.x / 2;
    double theta = euler.y / 2;
    double psi   = euler.z / 2;
    
    quat.w = cos(fi)*cos(theta)*cos(psi) + sin(fi)*sin(theta)*sin(psi);  
    quat.x = sin(fi)*cos(theta)*cos(psi) - cos(fi)*sin(theta)*sin(psi);
    quat.y = cos(fi)*sin(theta)*cos(psi) + sin(fi)*cos(theta)*sin(psi);
    quat.z = cos(fi)*cos(theta)*sin(psi) - sin(fi)*sin(theta)*cos(psi);
}

void Quat2Rota(geometry_msgs::Quaternion &quat, Eigen::Matrix3d& rot)
{
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;
    double w = quat.w;

    rot(0,0)=1-2*(y*y+z*z);
    rot(0,1)=2*(x*y-z*w);
    rot(0,2)=2*(x*z+y*w);

    rot(1,0)=2*(x*y+z*w);
    rot(1,1)=1-2*(x*x+z*z);
    rot(1,2)=2*(y*z-x*w);

    rot(2,0)=2*(x*z-y*w);
    rot(2,1)=2*(y*z+x*w);
    rot(2,2)=1-2*(x*x+y*y);

}

void Quat_ComPen_Cam2Tag(geometry_msgs::Quaternion& q)
{
    //cam coordinate and tag coordinate: tag coordinate is rotated -90 deg about the x axis of cam coordinate.
    // compensate the rotation, so rotate 90 deg. angle_compensate = 90, cos(angle_compensate/2) = sin(angle_compensate/2) = 0.707
    // x = 1*sin(angle_compensate/2), y = 0, z = 0, w = cos(angle_compensate/2);
    // quaternion1 * quaternion2 = " you can google ! " 
    
    double x = q.x;
    double y = q.y;
    double z = q.z;
    double w = q.w;

    q.x = 0.707 * (x + w);
    q.y = 0.707 * (y + z);
    q.z = 0.707 * (z - y);
    q.w = 0.707 * (w - x);

}