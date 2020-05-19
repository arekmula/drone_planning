// ROS
#include <ros/ros.h>
#include "../include/drone_planning/drone_planning.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


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


octomap_msgs::Octomap globalOctoMap;
sensor_msgs::PointCloud2 globalPointCloud;
nav_msgs::OccupancyGrid globalOccupancyMap;
visualization_msgs::MarkerArray markerArray;

void octomapCallback(const octomap_msgs::OctomapPtr& msg)
{
  /*!
  * Octomap callback function
  * commented lines are examples of getting to octomap data
  */
    globalOctoMap = *msg;

}

void pointCloudCallback(const sensor_msgs::PointCloud2Ptr& cloud){

    /*!
    * PointCloud2 callback function
    * commented lines are examples of getting to PointCloud data
    */
    globalPointCloud = *cloud;

}

void markerArrayCallback(const visualization_msgs::MarkerArrayPtr& mArray)
{
    /*!
    * markerArray callback function
    * commented lines are examples of getting to markerArray data
    */
    markerArray = *mArray;


}

void occupancyMapCallback(const nav_msgs::OccupancyGridPtr& oMap)
{
    /*!
    * occupancy map callback function
    * commented lines are examples of getting to occupancy data
    */

    globalOccupancyMap = *oMap;
}

int main(int argc, char **argv)
{
  /// init ROS node
  ros::init(argc, argv, "drone_planner");

  ///  create node handler
  ros::NodeHandle node("~");
  drone_planning::Planner3D planner_(node);

  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;

  /// setup ROS lopp rate
  ros::Rate loop_rate(1);

  /// subscribers to full octomap, octomap point cloud and markerArray
  ros::Subscriber octomapFull_sub = node.subscribe("/octomap_full", 10, octomapCallback);
  ros::Subscriber octmapPointCloud_sub = node.subscribe("/octomap_point_cloud_centers", 10, pointCloudCallback); // Wspolrzedne zajetych voxeli
  ros::Subscriber occupied_cells_vis_array_sub = node.subscribe("/occupied_cells_vis_array", 10, markerArrayCallback);
  ros::Subscriber occupancyMap_sub = node.subscribe("/projected_map", 10, occupancyMapCallback);

  /// path Publisher
  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("my_path",1000);


  while (ros::ok()){
      nav_msgs::Path plannedPath;

      /// calculating path
      if(globalOctoMap.data.size()>0) /// make sure that node has subsribed to octomap data
      {
          plannedPath = planner_.planPath(globalOctoMap);
          // std::cout<<plannedPath<<"\n";
          // std::cout<<"Sciezka \n";

      }

      /// publishing path
      path_pub.publish(plannedPath);

      /// TFs
      for (int i = 0 ; i < plannedPath.poses.size() ; i++)
      {
          //  std::cout<<" X : "<<plannedPath.poses[i].pose.position.x
          //   <<" Y : "<<plannedPath.poses[i].pose.position.y
          //  <<" Z : "<<plannedPath.poses[i].pose.position.z<<" \n";
          broadcaster.sendTransform(
                  tf::StampedTransform(
                          tf::Transform(tf::Quaternion(0, 0, 0, 1),
                                  tf::Vector3(-1, 0.5, 0.3)),
                          ros::Time::now(), "odom","drone"));

          broadcaster.sendTransform(
                  tf::StampedTransform(
                          tf::Transform(tf::Quaternion(0, 0, 0, 1),
                                  tf::Vector3(plannedPath.poses[i].pose.position.x + 1,
                                          plannedPath.poses[i].pose.position.y - 0.5,
                                          plannedPath.poses[i].pose.position.z - 0.3)),
                          ros::Time::now(), "drone","tf"));
          
          ros::Duration(1.0).sleep();
      }


      ros::spinOnce();
      loop_rate.sleep();
}
    return 0;
//  ros::spin();
}
