
#include "../include/drone_planning/drone_planning.hpp"

using namespace std;
using namespace ros;

namespace drone_planning{

Planner3D::Planner3D(ros::NodeHandle& _nodeHandle)
    : nodeHandle(_nodeHandle)
{
    ROS_INFO("STARTED PLANNING");
}

Planner3D::~Planner3D()
{
}

nav_msgs::Path Planner3D::planPath(const octomap_msgs::Octomap &globalOctoMap, const sensor_msgs::PointCloud2 &globalPointCloud)
{
    /// drawing test path
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

    return myPath;
}

}
