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
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/common/types.h>
#include <fcl/octree.h>
#include <fcl/data_types.h>
#include <fcl/math/vec_3f.h>
#include <fcl/math/math_details.h>

#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/collision_func_matrix.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/collision.h>
#include <ccd/ccd.h>
#include <ccd/quat.h>

#include <fcl/common/unused.h>
#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/octree/octree.h"

#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/continuous_collision_object.h"
#include "fcl/narrowphase/continuous_collision_request.h"
#include "fcl/narrowphase/continuous_collision_result.h"

#include "fcl/narrowphase/collision.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"



#include <cmath>
#include <limits>
#include <tf/transform_broadcaster.h>
#include <octomap_ros/conversions.h>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
#include <moveit/ompl_interface/ompl_interface.h>


#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/Planner.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <moveit/ompl_interface/ompl_interface.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>


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

#include <array>
#include <random>


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
    nav_msgs::Path planPath();

    /// get current start Position - > Useful in visualization
    void getStartPosition(float &xPos, float &yPos, float &zPos);


    /// configure
    void configure(const octomap_msgs::Octomap& octomapMsg);


private:

    /// node handle
    ros::NodeHandle& nodeHandle;

    /// problem dimension
    int dim;

    /// bounds for all dimensions
    std::shared_ptr<ompl::base::RealVectorBounds> bounds;

    /// starting and goal position
    std::shared_ptr<ompl::base::ScopedState<>> start;
    std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// intersting points to visit on our octomoap
    /// under pool table, in the little room, under table in biggest room, back room
    float intrestingX[4] = {4.92, 0.96,-2.75, -6.37};
    float intrestingY[4] = {2.80, 2.09, 2.42, -2.32};

    unsigned int intrestingPositionsNumber = 4;
    int curPosition = 0;

    /// space of problem
    std::shared_ptr<ompl::base::SE3StateSpace> space;
    /// space information
    std::shared_ptr<ompl::base::SpaceInformation> si;
    /// problem definition
    std::shared_ptr<ompl::base::ProblemDefinition> pdef;

    /* **************************************************************************
     * CHOOSE YOUR FIGHTER BELOW and in configure() method
     * Remember to check if you use single query planner or multi-query planner.
     * Remember to check if you imported library of used planner
     * Look for usage of clearQuery() and clear() functions in planPath() method.
     * *************************************************************************/
    /// planner definition
    std::shared_ptr<ompl::geometric::LazyPRMstar> planner;
    /* **************************************************************************
     * CHOOSE YOUR FIGHTER ABOVE
     * **************************************************************************/


    /// time which planner can use for searching and returning possible path
    double SOLVING_TIME = 3.0;

    /// get new random goal state
    void randomizeNewGoalState(void);

    /// save last start state to visualize movement of drone
    void saveStartState(void);

    /// extract path function
    nav_msgs::Path extractPath(ompl::base::ProblemDefinition* pdef);

    /// check approximate solution function
    bool checkApproximateSolution(ompl::base::ProblemDefinition* pdef, double errorThreshold);

    /// add intresting goal positions to vector
    void addIntrestingGoalPositions(void);

    /// initial move
    bool initialMove = true;

    /// Goal and start positions. Useful in visualization
    float xGoal, yGoal, zGoal;
    float xStart, yStart, zStart;


};

}
