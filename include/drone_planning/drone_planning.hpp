#pragma once

#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/ColorOcTree.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>


#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/common/types.h>
#include <fcl/octree.h>
#include <fcl/data_types.h>
#include <fcl/math/vec_3f.h>
#include <fcl/math/math_details.h>

#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/collision_geometry.h>

#include <cmath>
#include <limits>

#include <octomap_ros/conversions.h>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
#include <moveit/ompl_interface/ompl_interface.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <moveit/ompl_interface/ompl_interface.h>

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
#include <string>





namespace drone_planning{

class Planner3D
{
public:
    /*!
     * Constructor
     * @param nodeHandle the ros node Handle.
     */
    Planner3D(ros::NodeHandle& _nodeHandle);

    /*!
    * Destructor
    */
    virtual ~Planner3D();

    /*!
     * plan path
     * @param octomapMsg - Octomap message of enviroment
     */
    nav_msgs::Path planPath(const octomap_msgs::Octomap& octomapMsg);
    /// robot mesh points



private:
    /// node handle
    ros::NodeHandle& nodeHandle;

    /// problem dimension
    int dim;

    /// max step Length
    double maxStepLength;

    /// bounds for all dimensions
    std::shared_ptr<ompl::base::RealVectorBounds> bounds;

    /// starting and goal position
    std::shared_ptr<ompl::base::ScopedState<>> start;
    std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// space of problem
    std::shared_ptr<ompl::base::SE3StateSpace> space;

    /// configure
    void configure(void);

    /// extract path function
    nav_msgs::Path extractPath(ompl::base::ProblemDefinition* pdef);


};

}
