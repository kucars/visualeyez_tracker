/*************************************************************************************
* Copyright (C) 2013 - 2014 by                                                       *
* Tarek Taha, Rui P. de Figueiredo, Khalifa University Robotics Institute KURI       *
*                     <tarek.taha@kustar.ac.ae>, <rui.defigueiredo@kustar.ac.ae>     *
*                                                                                    *
*                                                                                    *
* This program is free software; you can redistribute it and/or modify               *
* it under the terms of the GNU General Public License as published by               *
* the Free Software Foundation; either version 2 of the License, or                  *
* (at your option) any later version.                                                *
*                                                                                    *
* This program is distributed in the hope that it will be useful,                    *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                     *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                       *
* GNU General Public License for more details.                                       *
*                                                                                    *
* You should have received a copy of the GNU General Public License                  *
* along with this program; if not, write to the                                      *
* Free Software Foundation, Inc.,                                                    *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.                        *
**************************************************************************************/
#include <endian.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <visualeyez_tracker/TrackerPose.h>

#include <visualeyez_tracker/PoseBroadcaster.h>

/*
  References used to develop this code:
  https://github.com/cwru-robotics/cwru-ros-pkg/blob/master/cwru_semi_stable/cwru_base/src/crio_receiver.cpp
  http://www.boost.org/doc/libs/1_46_0/doc/html/boost_asio/example/timeouts/async_tcp_client.cpp
  http://www.boost.org/doc/libs/1_46_1/doc/html/boost_asio/example/timeouts/blocking_tcp_client.cpp
  http://www.boost.org/doc/libs/1_46_0/doc/html/boost_asio/examples.html
  Remember that boost compatibility is important (methods and functions changed in the latest boost versions)!
  Boost 1.46.x is used in Ubuntu 12.04 so stick to it!
*/

using boost::asio::ip::tcp;
using boost::asio::deadline_timer;

void check_deadline(deadline_timer& deadline, tcp::socket& socket) 
{
    if (deadline.expires_at() <= deadline_timer::traits_type::now())
    {
        socket.cancel();
        deadline.expires_at(boost::posix_time::pos_infin);
    }
    deadline.async_wait(boost::bind(&check_deadline, boost::ref(deadline), boost::ref(socket)));
}

boost::asio::streambuf receiveBuffer;
bool gotTheData = false;
std::vector<std::string> tokens;

static void handle_receive(const boost::system::error_code& errorCode, std::size_t inputLength,boost::system::error_code* output_ec, std::size_t* outputLength)
{
    if(!errorCode)
    {
        tokens.clear();
        //ROS_INFO_STREAM("BUFFER SIZE:"<<receiveBuffer.size());fflush(stdout);
        std::istream is(&receiveBuffer);
        copy(std::istream_iterator<std::string>(is),
             std::istream_iterator<std::string>(),
             std::back_inserter<std::vector<std::string> >(tokens));

	/*std::cout<<"Got a new Line, number ot tokens:"<<is<<std::endl;fflush(stdout);        
        std::cout<<"Got a new Line, number ot tokens:"<<tokens.size()<<"\n";fflush(stdout);
        for(int i=0;i<tokens.size();i++)
        {
            if(tokens[i]!="" && tokens[i]!=" ")
                std::cout<<"Token:"<<tokens[i]<<"\n";fflush(stdout);
        }*/
     
        gotTheData = true;
    }
    else
        ROS_INFO_STREAM("Yes, they call me every now and Error Code: "<<output_ec->message());
}

void connect_handler(const boost::system::error_code& error)
{
    if (!error)
    {
        ROS_INFO("Successfully Connected");
    }
    else
    {
        ROS_INFO("Something went wrong");
    }
}

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "visualeyez_tracker");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");
    int socket_timeout;
    std::string server_ip,server_port;

    privateNh.param("socket_timeout", socket_timeout, int(10));
    privateNh.param<std::string>("server_ip",      server_ip,   std::string("10.10.101.50"));
    privateNh.param<std::string>("server_port",    server_port, std::string("12345"));
    ROS_INFO("Server Ip is:%s port is:%s socket timeout is:%d",server_ip.c_str(),server_port.c_str(),socket_timeout);

    visualeyez_tracker::TrackerPose trackerPose;
    ros::Publisher trackerPositionPublisher = nh.advertise<visualeyez_tracker::TrackerPose>("TrackerPosition", 100);



    boost::asio::io_service io_service;
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(tcp::v4(), server_ip.c_str(), server_port.c_str());
    tcp::resolver::iterator iterator = resolver.resolve(query);
    ROS_INFO("Trying to connect");
    tcp::socket socket(io_service);
    socket.async_connect(iterator->endpoint(), connect_handler);
    ROS_INFO("Connected");
    deadline_timer deadline(io_service);
    deadline.expires_at(boost::posix_time::pos_infin);
    check_deadline(deadline, socket);
    boost::posix_time::seconds timeout(socket_timeout);
    ros::Rate loop_rate(500);
    boost::system::error_code error;
    std::size_t length;

    PoseBroadcaster pose_broadcaster(nh);

    while (nh.ok())
    {
        try
        {
            error = boost::asio::error::would_block;
            length = 0;
            deadline.expires_from_now(timeout);

            boost::asio::async_read_until(socket, receiveBuffer, '\n', boost::bind(&handle_receive, _1, _2, &error, &length));
            while(!gotTheData)
            {
                io_service.poll();
            }
            gotTheData = false;


            if(((tokens.size()-1)%4)==0 && tokens.size()!=1)
            {
                int tuples = (tokens.size()/4);
		std::vector<visualeyez_tracker::TrackerPose> trackerPoses;
                for(int i=0;i<tuples;i++)
                {

                    trackerPose.header.stamp = ros::Time::now();
                    trackerPose.tracker_id   = tokens[4*i + 0];
                    trackerPose.pose.x       = atof(tokens[4*i + 1].c_str())/1000.0;
                    trackerPose.pose.y       = atof(tokens[4*i + 2].c_str())/1000.0;
                    trackerPose.pose.z       = atof(tokens[4*i + 3].c_str())/1000.0;
                    trackerPoses.push_back(trackerPose);
                    trackerPositionPublisher.publish(trackerPose);
                    //ROS_INFO(" VisualEyez Sending Location: [%s] [%f] [%f] [%f]",trackerPose.tracker_id.c_str(),trackerPose.pose.x ,trackerPose.pose.y ,trackerPose.pose.z );
                }


                pose_broadcaster.updateMarker(trackerPoses);
		//std::cout << "number of tuples:" << tuples << std::endl;
            }
        }
        catch (std::exception& e)
        {
            ROS_ERROR_STREAM("cRIO receiver threw an exception: " << e.what());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

