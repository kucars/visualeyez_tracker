/**********************************************************************************
* Copyright (C) 2013 - 2014 by                                                    *
* Rui P. de Figueiredo and Tarek Taha, Khalifa University Robotics Institute KURI *
*                 <rui.defigueiredo@kustar.ac.ae>,<tarek.taha@kustar.ac.ae>       *
*                                                                          	  *
*                                                                                 *
* This program is free software; you can redistribute it and/or modify     	  *
* it under the terms of the GNU General Public License as published by     	  *
* the Free Software Foundation; either version 2 of the License, or        	  *
* (at your option) any later version.                                      	  *
*                                                                          	  *
* This program is distributed in the hope that it will be useful,          	  *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           	  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             	  *
* GNU General Public License for more details.                              	  *
*                                                                          	  *
* You should have received a copy of the GNU General Public License        	  *
* along with this program; if not, write to the                            	  *
* Free Software Foundation, Inc.,                                          	  *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.              	  *
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
#define PI 3.14159265

double x,y,z;

const double epsilon=0.1;

inline bool equalFloat(double a, double b, double epsilon)
{
    return fabs(a - b) < epsilon;
}


class Robot
{
    ros::NodeHandle n_robot_priv;
    std::string robot_id_;

    std::map<std::string, int> marker_id_to_type;
    std::vector<Eigen::Vector3d> markers_position;
    std::vector<bool> markers_state;

    ros::Publisher pose_pub;
    ros::Publisher odom_pub;
    ros::NodeHandle n_;

    std::vector<Eigen::Vector3d> markers_offsets;

    int update_count_;


public:

    Robot(ros::NodeHandle & n,std::string robot_id,
          std::string origin_marker_id_,
          std::string x_marker_id_,
          std::string y_marker_id_);

    void updateMarkerPosition(std::string & marker_id, Eigen::Vector3d & position);

    void updateRobotPose();

    void flatTrim();
};


class PoseBroadcaster
{
public:

    std::map<std::string,boost::shared_ptr<Robot> > robots;

    bool got_pose_update_;
    bool got_base_marker_;
    bool got_head_marker_;
    bool got_side_marker_;


    Eigen::Vector3d base_marker_position;
    Eigen::Vector3d head_marker_position;
    Eigen::Vector3d side_marker_position;

    ros::Subscriber markers_sub;

    ros::NodeHandle n_;
    ros::ServiceServer add_robot_service_;

    PoseBroadcaster(ros::NodeHandle & n);
    bool addRobotTrackerCallback(visualeyez_tracker::AddRobotTracker::Request  &req,
                                 visualeyez_tracker::AddRobotTracker::Response &res);

    void updateMarker(const visualeyez_tracker::TrackerPose & trackerPose);
};

