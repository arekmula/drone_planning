
// ROS
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <octomap_ros/conversions.h>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
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



octomap_msgs::Octomap globalOctoMap; // To chyba przekazywac do funkcji z FCLa, bo to jest OcTree
sensor_msgs::PointCloud2 globalPointCloud; // Wspolrzedne zajetych voxeli
visualization_msgs::MarkerArray markerArray;

void octomapCallback(const octomap_msgs::OctomapPtr& octMap)
{

    globalOctoMap = *octMap;
    std::cout << "   " << "\n";
    std::cout << "globalMap Resolution: " << globalOctoMap.resolution << "\n";
    std::cout << "globalMap Binary: " << globalOctoMap.binary <<"\n";
    std::cout << "globalMap id: " << globalOctoMap.id << "\n";
    std::cout << "globalMap frameID: " << globalOctoMap.header.frame_id << "\n";



    //    for(int i=0; i < globalMap.data.size(); i++){
    //        std::cout << int(globalMap.data[i]) << "\n";
    //    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2Ptr& cloud){

//    std::cout<<int(cloud.data[0])<< "\n";



    globalPointCloud = *cloud;
    std::cout << "   " << "\n";
    std::cout << "PointCloudFrameID: " << globalPointCloud.header.frame_id << "\n";
    std::cout << "PointCloudHeight: " << globalPointCloud.height << "\n";  ///Punkty sa unordered po height=1
    std::cout << "PointCloudWidth: " << globalPointCloud.width << "\n";



//    std::cout << int(globalPointCloud.data[0]) << "\n";
//    BOOST_FOREACH(const pcl::PointXYZ& pt, globalPointCloud.data)
}

void markerArrayCallback(const visualization_msgs::MarkerArrayPtr& mArray)
{
    markerArray = *mArray;


//    std::cout << "markerArray.markers.size: " << markerArray.markers.size() << "\n";
//    std::cout << "markerArray.markers[0].points.size(): " << markerArray.markers[0].points.size() << "\n";
//    for (int i=0; i<markerArray.markers.size(); i++)
//    {
//            std::cout << "markerArray.markers[i].points.size():" << markerArray.markers[i].points.size() << "\n";
//    }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_planner");
  ros::NodeHandle node("~");

  ros::Rate loop_rate(1);


  ros::Subscriber octomapFull_sub = node.subscribe("/octomap_full", 10, octomapCallback);
  ros::Subscriber octmapPointCloud_sub = node.subscribe("/octomap_point_cloud_centers", 10, pointCloudCallback); // Wspolrzedne zajetych voxeli
  ros::Subscriber occupied_cells_vis_array_sub = node.subscribe("/occupied_cells_vis_array", 10, markerArrayCallback);

  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("my_path",1000);

  while (ros::ok()){
      nav_msgs::Path myPath;
      myPath.header.stamp = ros::Time::now();
      myPath.header.frame_id = "odom";

      geometry_msgs::PoseStamped pose1;
      pose1.pose.position.x = 0;
      pose1.pose.position.y = 0;
      pose1.pose.position.z = 0;
      pose1.pose.orientation.x = 0;
      pose1.pose.orientation.y = 0;
      pose1.pose.orientation.z = 0;
      pose1.pose.orientation.w = 1;
      pose1.header.frame_id = "/odom";
      pose1.header.stamp = ros::Time::now();

      myPath.poses.push_back(pose1);

      geometry_msgs::PoseStamped pose2;
      pose2.pose.position.x = 0;
      pose2.pose.position.y = 0;
      pose2.pose.position.z = 1;
      pose2.pose.orientation.x = 0;
      pose2.pose.orientation.y = 0;
      pose2.pose.orientation.z = 0;
      pose2.pose.orientation.w = 1;
      pose2.header.frame_id = "/odom";
      pose2.header.stamp = ros::Time::now();
      myPath.poses.push_back(pose2);

      path_pub.publish(myPath);
      ros::spinOnce();
      loop_rate.sleep();
}
    return 0;
//  ros::spin();
}
