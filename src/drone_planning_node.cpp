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

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

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
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
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
  ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 1000 );


  while (ros::ok()){
      nav_msgs::Path plannedPath;

      /// Visualization marker
      visualization_msgs::Marker marker;
      marker.header.frame_id = "tf";
      marker.header.stamp = ros::Time();
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.mesh_resource = "package://drone_planning/meshes/quadrotor/quadrotor_2.dae";



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
          geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
          nav_msgs::Odometry odom;
          odom.header.stamp = ros::Time::now();
          odom.header.frame_id = "odom";

          //set the position
          odom.pose.pose.position.x = 0;
          odom.pose.pose.position.y = 0;
          odom.pose.pose.position.z = 0.0;
          odom.pose.pose.orientation = odom_quat;

          //set the velocity
          odom.child_frame_id = "drone";
          odom.twist.twist.linear.x = 0;
          odom.twist.twist.linear.y = 0;
          odom.twist.twist.angular.z = 0;

          //publish the message
          odom_pub.publish(odom);

          /// Drone marek pose
          marker.pose.orientation.x = plannedPath.poses[i].pose.position.x;
          marker.pose.orientation.y = plannedPath.poses[i].pose.position.y;
          marker.pose.orientation.z = plannedPath.poses[i].pose.position.z;

          /// Drone marker orientation
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          /// ADD DRONE
          marker.action = visualization_msgs::Marker::ADD;

          /// publishing marker
          vis_pub.publish(marker);

          ros::Duration(1.0).sleep();

          /// DELETE DRONE
          marker.action = visualization_msgs::Marker::DELETE;
          
      }


      ros::spinOnce();
      loop_rate.sleep();
}
    return 0;
//  ros::spin();
}
