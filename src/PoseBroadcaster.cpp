/**********************************************************************************
* Copyright (C) 2013 - 2014 by                                                    *
* Rui P. de Figueiredo and Tarek Taha, Khalifa University Robotics Institute KURI *
*                 <rui.defigueiredo@kustar.ac.ae>, <tarek.taha@kustar.ac.ae>      *
*                                                                          	      *
*                                                                                 *
* This program is free software; you can redistribute it and/or modify     	      *
* it under the terms of the GNU General Public License as published by     	      *
* the Free Software Foundation; either version 2 of the License, or        	      *
* (at your option) any later version.                                      	      *
*                                                                          	      *
* This program is distributed in the hope that it will be useful,          	      *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           	      *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             	      *
* GNU General Public License for more details.                              	  *
*                                                                          	      *
* You should have received a copy of the GNU General Public License        	      *
* along with this program; if not, write to the                            	      *
* Free Software Foundation, Inc.,                                          	      *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.              	      *
***********************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "pctx_control/Control.h"
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
    update_count_(0)
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

    pose_pub=n_robot_priv.advertise<geometry_msgs::PoseStamped>("pose", 100);
    odom_pub=n_.advertise<nav_msgs::Odometry>("/ground_truth/state", 100);
}

void Robot::updateMarkerPosition(std::string & marker_id, Eigen::Vector3d & position)
{
    // Get marker type from id
    int type=marker_id_to_type.find(marker_id)->second;

    // Update position
    markers_position[type]=position;

    // Update state
    markers_state[type]=true;
    //        std::cout << "robot:" << robot_id_ <<" marker update:" << marker_id << " position:" << position.transpose() << std::endl;

    for(int i=0; i<markers_position.size(); ++i)
    {
        if(!markers_state[i])
            return;
    }

    if(update_count_==0)
    {
        flatTrim();
    }
    updateRobotPose();
    ++update_count_;

}

void Robot::updateRobotPose()
{
    //Base - 0
    //X    - 1
    //Y    - 2
    // Define a local reference frame
    Eigen::Vector3d x_axis=((markers_position[1]+markers_offsets[1])-(markers_position[0]+markers_offsets[0])).normalized(); // Heading
    Eigen::Vector3d z_axis=x_axis.cross( ( (markers_position[2]+markers_offsets[2])-(markers_position[1]+markers_offsets[1]) ).normalized() ).normalized();
    Eigen::Vector3d y_axis=z_axis.cross(x_axis).normalized();
    //        std::cout << "x_axis: "<<x_axis.transpose() <<  " y_axis: "<<y_axis.transpose() << " z_axis: "<<z_axis.transpose() << std::endl;
    Eigen::Matrix<double, 3, 3> rotation_matrix;
    rotation_matrix << x_axis, y_axis, z_axis;
    Eigen::Quaternion<double> quaternion(rotation_matrix);

    /////////////////////////
    // Publish PoseStamped //
    /////////////////////////

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp=ros::Time::now();
    //pose_msg.header.frame_id=robot_id_;
    pose_msg.header.frame_id="world";

    pose_msg.pose.position.x=markers_position[0].x();
    pose_msg.pose.position.y=markers_position[0].y();
    pose_msg.pose.position.z=markers_position[0].z();
    pose_msg.pose.orientation.w=quaternion.w();
    pose_msg.pose.orientation.x=quaternion.x();
    pose_msg.pose.orientation.y=quaternion.y();
    pose_msg.pose.orientation.z=quaternion.z();
    pose_pub.publish(pose_msg);

    //////////////////
    // Broadcast TF //
    //////////////////
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin( tf::Vector3(markers_position[0].x(), markers_position[0].y(), markers_position[0].z()) );
    tf::Quaternion q(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_id_));

    //////////////////////
    // Publish odometry //
    //////////////////////

    nav_msgs::Odometry odom_msg;
    odom_msg.header=pose_msg.header;
    odom_msg.pose.pose=pose_msg.pose;
    odom_pub.publish(odom_msg);

    for(int i=0; i<markers_state.size(); ++i)
    {
        markers_state[i]=false;
    }
}

void Robot::flatTrim()
{
    ROS_INFO("Auto flat trim...");



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

    //markers_sub = n_.subscribe("/TrackerPosition", 100, &PoseBroadcaster::poseFromMarkers, this);
}

bool PoseBroadcaster::addRobotTrackerCallback(visualeyez_tracker::AddRobotTracker::Request  &req,
                                              visualeyez_tracker::AddRobotTracker::Response &res)
{
    boost::shared_ptr<Robot> RobotPtr(new Robot(n_,req.robot_name,req.origin_marker_id,req.x_marker_id,req.y_marker_id));
    robots.insert(std::map<std::string, boost::shared_ptr<Robot> >::value_type(req.origin_marker_id, RobotPtr));
    robots.insert(std::map<std::string, boost::shared_ptr<Robot> >::value_type(req.x_marker_id, RobotPtr));
    robots.insert(std::map<std::string, boost::shared_ptr<Robot> >::value_type(req.y_marker_id, RobotPtr));


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
        //std::cout <<  "found:" <<marker_id << std::endl;
        //element found;
        it->second->updateMarkerPosition(marker_id,marker_position);
    }
    //else
    //std::cout <<  "not found:" <<marker_id << std::endl;
}


