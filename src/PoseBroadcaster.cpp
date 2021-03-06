/**********************************************************************************
* Copyright (C) 2013 - 2014 by                                                    *
* Rui P. de Figueiredo and Tarek Taha, Khalifa University Robotics Institute KURI *
*                 <rui.defigueiredo@kustar.ac.ae>, <tarek.taha@kustar.ac.ae>      *
*                                                                                 *
*                                                                                 *
* This program is free software; you can redistribute it and/or modify     	  *
* it under the terms of the GNU General Public License as published by            *
* the Free Software Foundation; either version 2 of the License, or               *
* (at your option) any later version.                                             *
*                                                                                 *
* This program is distributed in the hope that it will be useful,                 *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                    *
* GNU General Public License for more details.                              	  *
*                                                                                 *
* You should have received a copy of the GNU General Public License               *
* along with this program; if not, write to the                                   *
* Free Software Foundation, Inc.,                                                 *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.                     *
***********************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualeyez_tracker/TrackerPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <map>
#include "visualeyez_tracker/AddRobotTracker.h"
#include "visualeyez_tracker/PoseBroadcaster.h"
#define PI 3.14159265


Robot::Robot(ros::NodeHandle & n,std::string robot_id,
             std::string origin_marker_id_,
             std::string x_marker_id_,
             std::string y_marker_id_):
    n_(n),
    robot_id_(robot_id),
    n_robot_priv(robot_id),
    update_count_(0),
    marker_change(false)
{
    marker_id_to_type[origin_marker_id_]=0;
    marker_id_to_type[x_marker_id_]=1;
    marker_id_to_type[y_marker_id_]=2;

    markers_state.push_back(false);
    markers_state.push_back(false);
    markers_state.push_back(false);
    markers_position.resize(3);
    markers_offsets.resize(3);
    for(int i=0; i<markers_offsets.size();++i)
    {
        markers_offsets[i]=Eigen::Vector3d(0.0,0.0,0.0);
    }

    pose_nwu_pub      = n_robot_priv.advertise<geometry_msgs::PoseStamped>("pose_NWU", 100);
    pose_enu_pub      = n_robot_priv.advertise<geometry_msgs::PoseStamped>("pose_ENU", 100);
}

void Robot::updateMarkerPosition(std::string & marker_id, Eigen::Vector3d & position)
{
    marker_change = true;
    // Get marker type from id
    int type=marker_id_to_type.find(marker_id)->second;

    // Update position
    markers_position[type]=position;

    // Update state
    markers_state[type]=true;
    for(int i=0; i<markers_position.size(); ++i)
    {
        if(!markers_state[i])
        {    
            return;
        }
    }

    if(update_count_==0)
    {
        flatTrim();
    }
    //updateRobotPose();
    ++update_count_;

}

void Robot::updateRobotPose()
{
    if(!marker_change)
	return;
    //Base - 0
    //X    - 1
    //Y    - 2
    // Define a local reference frame
    Eigen::Vector3d x_axis = ((markers_position[1]+markers_offsets[1])-(markers_position[0]+markers_offsets[0])).normalized(); // Heading
    Eigen::Vector3d z_axis = x_axis.cross( ( (markers_position[2]+markers_offsets[2])-(markers_position[1]+markers_offsets[1]) ).normalized() ).normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    static tf::TransformBroadcaster br;    
    tf::Transform transform;
    geometry_msgs::PoseStamped pose_msg;
    Eigen::Matrix<double, 3, 3> rotation_matrix;
    rotation_matrix << x_axis, y_axis, z_axis;
    Eigen::Quaternion<double> quaternion(rotation_matrix);
    // q is needed for the ROS internal tf functions
    tf::Quaternion q(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
    // Get the YPR from the current Quetrenion
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);  

    transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion qt = tf::createQuaternionFromRPY(0,0,0);
    transform.setRotation(qt);  
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_id_+"/NWU_world"));  
    
    qt = tf::createQuaternionFromRPY(0,0,-PI/2.0f);
    transform.setRotation(qt);  
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_id_+"/ENU_world"));  
    
    // Correct the heading (due to the fact that the marker placement is skew by 45 degrees) 
    // by 45 degrees CCW = +ve addition to yaw (rotation around z-axis)    
    qt = tf::createQuaternionFromYaw(yaw + PI/4.0f);
    pose_msg.header.stamp       = ros::Time::now();
    pose_msg.header.frame_id    = "world";
    // I can't seem to find the function that transfroms a pose using tf::transform
    // so it's hard coded for now in the ENU frame
    /*
    pose_msg.pose.position.x    =-markers_position[0].y();
    pose_msg.pose.position.y    = markers_position[0].x();
    pose_msg.pose.position.z    = markers_position[0].z();
    */
    pose_msg.pose.position.x    = markers_position[0].x();
    pose_msg.pose.position.y    = markers_position[0].y();
    pose_msg.pose.position.z    = markers_position[0].z();    
    pose_msg.pose.orientation.w = qt.w();
    pose_msg.pose.orientation.x = qt.x();
    pose_msg.pose.orientation.y = qt.y();
    pose_msg.pose.orientation.z = qt.z();    
    pose_enu_pub.publish(pose_msg);
    
    /*
    pose_msg.header.stamp       = ros::Time::now();
    pose_msg.header.frame_id    = "world";
    pose_msg.pose.position.x    =-markers_position[0].y();
    pose_msg.pose.position.y    = markers_position[0].x();
    pose_msg.pose.position.z    = markers_position[0].z();
    pose_msg.pose.orientation.w = quaternion.w();
    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();    
    pose_enu_pub.publish(pose_msg);
    */
    
    ////////////////////////////
    // Publish PoseStamped NWU//
    ////////////////////////////
    transform.setOrigin( tf::Vector3(markers_position[0].x(), markers_position[0].y(), markers_position[0].z()) );
    // Correct the heading (due to the fact that the marker placement is skew by 45 degrees) 
    // by 45 degrees CCW = +ve addition to yaw (rotation around z-axis)
    tf::Quaternion correctedQuaternionNWM = tf::createQuaternionFromYaw(PI/4.0f);
    transform.setRotation(correctedQuaternionNWM);  
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_id_+"/baselink_NWU"));    
    
    pose_msg.header.stamp       = ros::Time::now();
    pose_msg.header.frame_id    = "world";
    pose_msg.pose.position.x    = markers_position[0].x();
    pose_msg.pose.position.y    = markers_position[0].y();
    pose_msg.pose.position.z    = markers_position[0].z();
    pose_msg.pose.orientation.w = correctedQuaternionNWM.w();
    pose_msg.pose.orientation.x = correctedQuaternionNWM.x();
    pose_msg.pose.orientation.y = correctedQuaternionNWM.y();
    pose_msg.pose.orientation.z = correctedQuaternionNWM.z();
    pose_nwu_pub.publish(pose_msg);    
    
    ////////////////////////////
    // Publish PoseStamped ENU//
    ////////////////////////////
    transform.setOrigin( tf::Vector3(-markers_position[0].y(), markers_position[0].x(), markers_position[0].z()) );    
    // To change from NWU to ENU we have to rotate 90 degrees CW (-ve) around z-axis = yaw
    // This has to be done after correcting the orinetaiton offse of 45 degrees CCW : Total=> -90 + 45 = -45 degrees
    tf::Quaternion correctedQuaternionENU = tf::createQuaternionFromYaw(yaw + PI/4.0f + PI/2.0f);    
    transform.setRotation(correctedQuaternionENU);      
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_id_+"/baselink_ENU"));
    
    /*
    transform.setOrigin( tf::Vector3(markers_position[0].x(), markers_position[0].y(), markers_position[0].z()) );    
    // To change from NWU to ENU we have to rotate 90 degrees CW (-ve) around z-axis = yaw
    // This has to be done after correcting the orinetaiton offse of 45 degrees CCW : Total=> -90 + 45 = -45 degrees
    tf::Quaternion correctedQuaternionENU = tf::createQuaternionFromYaw(yaw -PI/4.0f);    
    transform.setRotation(correctedQuaternionENU);      
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_id_+"/baselink_ENU"));
    */
   
    marker_change=false;
}

void Robot::flatTrim()
{
    ROS_INFO("Auto flat trim... (removing z-offset of markers to make them planner)");

    markers_offsets[1](2,0)=markers_position[0](2,0)-markers_position[1](2,0);
    markers_offsets[2](2,0)=markers_position[0](2,0)-markers_position[2](2,0);
    
    std::cout << markers_offsets[0].transpose() << std::endl;
    std::cout << markers_offsets[1].transpose() << std::endl;
    std::cout << markers_offsets[2].transpose() << std::endl;
    ROS_INFO("Done.");
}

PoseBroadcaster::PoseBroadcaster(ros::NodeHandle & n) :
    n_(n)
{
    add_robot_service_ = n.advertiseService("add_robot_tracker", &PoseBroadcaster::addRobotTrackerCallback, this);
}

bool PoseBroadcaster::addRobotTrackerCallback(visualeyez_tracker::AddRobotTracker::Request  &req,
                                              visualeyez_tracker::AddRobotTracker::Response &res)
{
    std::string Ch = "Channel";
    req.origin_marker_id = Ch + req.origin_marker_id;
    req.x_marker_id	 = Ch + req.x_marker_id;
    req.y_marker_id      = Ch + req.y_marker_id;

    boost::shared_ptr<Robot> RobotPtr(new Robot(n_,req.robot_name,req.origin_marker_id,req.x_marker_id,req.y_marker_id));
    robots.insert(std::map<std::string, boost::shared_ptr<Robot> >::value_type( req.origin_marker_id, RobotPtr));
    robots.insert(std::map<std::string, boost::shared_ptr<Robot> >::value_type( req.x_marker_id, RobotPtr));
    robots.insert(std::map<std::string, boost::shared_ptr<Robot> >::value_type( req.y_marker_id, RobotPtr));
    std::cout << "inserted new robot with id:" << req.robot_name<< " and markers (base marker: " << req.origin_marker_id << ", x marker: "<< req.x_marker_id << ", y marker: " << req.y_marker_id<< ")"<< std::endl;
    return true;
}

void PoseBroadcaster::updateMarker(const visualeyez_tracker::TrackerPose & trackerPose)
{
    std::string marker_id(std::string(trackerPose.tracker_id));
    Eigen::Vector3d marker_position(trackerPose.pose.x, trackerPose.pose.y, trackerPose.pose.z);

    std::map<std::string,boost::shared_ptr<Robot> >::iterator it=robots.find(marker_id);
    if(it != robots.end())
    {
        it->second->updateMarkerPosition(marker_id,marker_position);
    }
}

void PoseBroadcaster::updateMarker(const std::vector<visualeyez_tracker::TrackerPose>  &trackerPoses)
{
    for(int i=0;i<trackerPoses.size();i++)
    {
	    std::string marker_id(std::string(trackerPoses[i].tracker_id));
	    Eigen::Vector3d marker_position(trackerPoses[i].pose.x, trackerPoses[i].pose.y, trackerPoses[i].pose.z);

	    std::map<std::string,boost::shared_ptr<Robot> >::iterator it=robots.find(marker_id);
	    if(it != robots.end())
	    {
		it->second->updateMarkerPosition(marker_id,marker_position);
	    }
    }

    for(std::map<std::string,boost::shared_ptr<Robot> >::iterator it=robots.begin(); it !=robots.end(); ++it)
    {
	it->second->updateRobotPose();
    }
}

