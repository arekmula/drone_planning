
// ROS
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// Octomap conversions
#include <drone_planning/conversions.h>

// Boost
#include <boost/thread.hpp>

//standard
#include <mutex>
#include <iostream>
#include <thread>
#include <fstream>
#include <iomanip>



octomap_msgs::Octomap globalMap;


void octomapCallback(const octomap_msgs::OctomapPtr& octMap)
{

    globalMap = *octMap;
    std::cout << "globalMap Resolution: " << globalMap.resolution << "\n";
    std::cout << "globalMap Binary: " << globalMap.binary <<"\n";
    std::cout << "globalMap id: " << globalMap.id << "\n";
    //    for(int i=0; i < globalMap.data.size(); i++){
    //        std::cout << int(globalMap.data[i]) << "\n";
    //    }
}

void octomapCloud_listener(const sensor_msgs::PointCloud2 &cloud){

//    std::cout<<int(cloud.data[0])<< "\n";

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "nswr_gps");
  ros::NodeHandle node("~");


  // GPS Fix subscribers
  ros::Subscriber octomapFull_sub = node.subscribe("/octomap_full", 10, octomapCallback);
  ros::Subscriber octmapPointCloud_sub = node.subscribe("/octomap_point_cloud_centers", 10, octomapCloud_listener);



  ros::spin();
}
