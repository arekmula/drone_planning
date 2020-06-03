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
visualization_msgs::Marker marker;

void octomapCallback(const octomap_msgs::OctomapPtr& msg)
{
    globalOctoMap = *msg;
}

void moveDrone(const nav_msgs::Path& plannedPath, const ros::Publisher& odom_pub,tf::TransformBroadcaster& broadcaster,
               const ros::Publisher& vis_pub, drone_planning::Planner3D planner_)
{
    /// Visualization marker
    marker.header.frame_id = "odom";
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
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.mesh_resource = "package://drone_planning/meshes/quadrotor/quadrotor_2.dae";

    /// Start point
    float x_prev =0.0 , y_prev = 0.0 , z_prev = 0.0;
    planner_.getStartPosition(x_prev, y_prev, z_prev);
    std::cout << "Visualized start state: " << x_prev << " " << y_prev << " " << z_prev << "\n";


    /// TFs
    for (int i = 0 ; i < plannedPath.poses.size() ; i++)
    {
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0, 1),
                                      tf::Vector3(x_prev, y_prev, z_prev)),
                        ros::Time::now(), "odom","drone"));

        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(tf::Quaternion(plannedPath.poses[i].pose.orientation.x,
                                                     plannedPath.poses[i].pose.orientation.y,
                                                     plannedPath.poses[i].pose.orientation.z,
                                                     plannedPath.poses[i].pose.orientation.w),
                                      tf::Vector3(plannedPath.poses[i].pose.position.x + x_prev,
                                                  plannedPath.poses[i].pose.position.y - y_prev,
                                                  plannedPath.poses[i].pose.position.z - z_prev)),
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
        marker.pose.position.x = plannedPath.poses[i].pose.position.x;
        marker.pose.position.y = plannedPath.poses[i].pose.position.y;
        marker.pose.position.z = plannedPath.poses[i].pose.position.z;

        /// Drone marker orientation
        marker.pose.orientation.x = plannedPath.poses[i].pose.orientation.x;
        marker.pose.orientation.y = plannedPath.poses[i].pose.orientation.y;
        marker.pose.orientation.z = plannedPath.poses[i].pose.orientation.z;
        marker.pose.orientation.w = plannedPath.poses[i].pose.orientation.w;

        /// Distance beetwen points
        int number_of_stamples = 100;
        float delay = 0.05;
        float distance = sqrt(pow(plannedPath.poses[i].pose.position.x - x_prev , 2)  +
                              pow(plannedPath.poses[i].pose.position.y - y_prev , 2) +
                              pow(plannedPath.poses[i].pose.position.z - z_prev , 2));

        float step_x, step_y, step_z;
        step_x = abs(plannedPath.poses[i].pose.position.x - x_prev) / number_of_stamples;
        step_y = abs(plannedPath.poses[i].pose.position.y - y_prev) / number_of_stamples;
        step_z = abs(plannedPath.poses[i].pose.position.z - z_prev) / number_of_stamples;

        float X = x_prev , Y = y_prev, Z = z_prev;
        /// X
        if(X < plannedPath.poses[i].pose.position.x)
        {
            X  = (step_x * abs(plannedPath.poses[i].pose.position.x - x_prev))/ distance + x_prev;
        }
        if(X > plannedPath.poses[i].pose.position.x)
        {
            X  = -1 * (step_x * abs(plannedPath.poses[i].pose.position.x - x_prev))/ distance + x_prev;
        }
        /// Y
        if(Y < plannedPath.poses[i].pose.position.y)
        {
            Y  = (step_y * abs(plannedPath.poses[i].pose.position.y - y_prev))/ distance + y_prev;
        }
        if(Y > plannedPath.poses[i].pose.position.y)
        {
            Y  = -1 * (step_y * abs(plannedPath.poses[i].pose.position.y - y_prev))/ distance + y_prev;
        }
        /// Z
        if(Z < plannedPath.poses[i].pose.position.z)
        {
            Z  = (step_z * abs(plannedPath.poses[i].pose.position.z - z_prev))/ distance + z_prev;
        }
        if(Z > plannedPath.poses[i].pose.position.z)
        {
            Z  = -1 * (step_z * abs(plannedPath.poses[i].pose.position.z - z_prev))/ distance + z_prev;
        }



        for (int j = 0; j < number_of_stamples; j++)
        {
            /// X
            if(X < plannedPath.poses[i].pose.position.x)
            {X  = X + step_x;}
            if(X > plannedPath.poses[i].pose.position.x)
            {X  = X - step_x;}
            /// Y
            if(Y < plannedPath.poses[i].pose.position.y)
            {Y  = Y + step_y;}
            if(Y > plannedPath.poses[i].pose.position.y)
            {Y  = Y - step_y;}
            /// Z
            if(Z < plannedPath.poses[i].pose.position.z)
            {Z  = Z + step_z;}
            if(Z > plannedPath.poses[i].pose.position.z)
            {Z  = Z - step_z;}

            marker.pose.position.x = X;
            marker.pose.position.y = Y;
            marker.pose.position.z = Z;
            /// ADD DRONE
            marker.action = visualization_msgs::Marker::ADD;

            /// publishing marker drone
            vis_pub.publish(marker);

            /// Delay
            ros::Duration(delay).sleep();

            /// DELETE DRONE
            marker.action = visualization_msgs::Marker::DELETE;
        }

        x_prev =  plannedPath.poses[i].pose.position.x;
        y_prev =  plannedPath.poses[i].pose.position.y;
        z_prev =  plannedPath.poses[i].pose.position.z;

    }

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
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster broadcaster;
    ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 1000 );


    /// subscribers to full octomap, octomap point cloud and markerArray
    ros::Subscriber octomapFull_sub = node.subscribe("/octomap_full", 10, octomapCallback);

    /// path Publisher
    ros::Publisher path_pub = node.advertise<nav_msgs::Path>("my_path",1000);


    while (ros::ok()){
        nav_msgs::Path plannedPath;

        /// calculating path
        if(globalOctoMap.data.size()>0) /// make sure that node has subsribed to octomap data
        {
            /// finding path
            plannedPath = planner_.planPath(globalOctoMap);
            /// publishing path
            path_pub.publish(plannedPath);
            /// moving drone
            moveDrone(plannedPath,odom_pub, broadcaster, vis_pub, planner_);
        }

        ros::spinOnce();
    }
    return 0;
}
