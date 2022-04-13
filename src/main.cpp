/*
 * main.cpp
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <std_msgs/Float32MultiArray.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace std;

#define param_size 10

class ros_node_class
{
private:
    ros::NodeHandle _nh;
    //ros::Publisher pcl_pub, solo_setting_pub, formation_setting_pub;
    std::string _file_location;

public:

    double f_v[param_size], s_v[param_size];
    ros::Publisher pcl_pub, solo_setting_pub, formation_setting_pub;

    ros_node_class(ros::NodeHandle &nodeHandle)
    {
        pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/param/pcl", 100, true);
        formation_setting_pub = _nh.advertise<std_msgs::Float32MultiArray>("/param/formation_settings", 100, true);
        solo_setting_pub = _nh.advertise<std_msgs::Float32MultiArray>("/param/solo_settings", 100, true);

        std::string node_name = ros::this_node::getName();
        _nh.param<std::string>("/" + node_name + "/pcd_file_location", _file_location, "/cloud.pcd");

        _nh.param<double>("/" + node_name + "/formation/step_size", f_v[0], 1.0);
        _nh.param<double>("/" + node_name + "/solo/step_size", s_v[0], 1.0);

        _nh.param<double>("/" + node_name + "/formation/obs_threshold", f_v[1], 1.0);
        _nh.param<double>("/" + node_name + "/solo/obs_threshold", s_v[1], 1.0);

        _nh.param<double>("/" + node_name + "/formation/xybuffer", f_v[2], 1.0);
        _nh.param<double>("/" + node_name + "/solo/xybuffer", s_v[2], 1.0);

        _nh.param<double>("/" + node_name + "/formation/zbuffer", f_v[3], 1.0);
        _nh.param<double>("/" + node_name + "/solo/zbuffer", s_v[3], 1.0);

        _nh.param<double>("/" + node_name + "/formation/passage_size", f_v[4], 1.0);
        _nh.param<double>("/" + node_name + "/solo/passage_size", s_v[4], 1.0);

        _nh.param<double>("/" + node_name + "/formation/min_height", f_v[5], 1.0);
        _nh.param<double>("/" + node_name + "/solo/min_height", s_v[5], 1.0);

        _nh.param<double>("/" + node_name + "/formation/max_height", f_v[6], 1.0);
        _nh.param<double>("/" + node_name + "/solo/max_height", s_v[6], 1.0);

        _nh.param<double>("/" + node_name + "/formation/max_tries", f_v[7], 1.0);
        _nh.param<double>("/" + node_name + "/solo/max_tries", s_v[7], 1.0);

        _nh.param<double>("/" + node_name + "/formation/timeout", f_v[8], 1.0);
        _nh.param<double>("/" + node_name + "/solo/timeout", s_v[8], 1.0);

        _nh.param<double>("/" + node_name + "/formation/z_scale", f_v[9], 1.0);
        _nh.param<double>("/" + node_name + "/solo/z_scale", s_v[9], 1.0);

        printf("%s[main.cpp] Constructor Setup Ready! \n", KGRN);
    }
    ~ros_node_class(){};

    void pcl_publisher() 
    {
        // Convert point cloud from pcl point ptr to ROS sensor message
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // Initialize the point cloud
    
        std::ifstream file(_file_location);
    	if(!file.good())            // If the file was not found, then file is 0, i.e. !file=1 or true.
	{
	    printf("%s[main.cpp] No pcd file found\n", KRED);
	    return;
	}
	else
	{
            pcl::io::loadPCDFile<pcl::PointXYZ>(_file_location, *cloud);// Load the pcd file
            printf("%s[main.cpp] Loaded .pcd cloud from %s\n", KGRN, _file_location.c_str());

            sensor_msgs::PointCloud2 full_cloud;
            pcl::toROSMsg(*cloud, full_cloud);
            printf("%s[main.cpp] Completed cloud to ROS message\n", KGRN);

            full_cloud.header.stamp = ros::Time::now();
            full_cloud.header.frame_id = "/map_nwu";
            pcl_pub.publish(full_cloud); 
        }
    }

    void rrt_param_publisher()
    {
        std_msgs::Float32MultiArray formation, solo;
        for (int i = 0; i < param_size; i++)
        {
            formation.data.push_back(f_v[i]);
            solo.data.push_back(s_v[i]);
	        printf("%s[main.cpp] %d formation %lf and solo %lf\n", KGRN, i, f_v[i], s_v[i]);
        }

        formation_setting_pub.publish(formation);
        solo_setting_pub.publish(solo);
    }

};

int main(int argc, char **argv)
{
    int count = 0;
    ros::init(argc, argv, "swarm_param_node");
    ros::NodeHandle _nh("~"); 
    ros_node_class ros_node_class(_nh);

    //while (ros_node_class.formation_setting_pub.getNumSubscribers() == 0)
	//ros::Duration(1).sleep();

    //while (ros_node_class.solo_setting_pub.getNumSubscribers() == 0)
	//ros::Duration(1).sleep();

    while(ros::ok() && count < 3)
    {
        ros_node_class.pcl_publisher();
        ros_node_class.rrt_param_publisher();

        ros::spinOnce();

        ros::Duration(0.5).sleep();
        count++;
    }

    return 0;
}
