
#include "../include/drone_planning/drone_planning.hpp"

using namespace std;
using namespace ros;

namespace drone_planning{

Planner3D::Planner3D(ros::NodeHandle& _nodeHandle)
    : nodeHandle(_nodeHandle)
{
    ROS_INFO("STARTED PLANNING");
    configure();
}

Planner3D::~Planner3D()
{
}

nav_msgs::Path Planner3D::examplePath(const octomap_msgs::Octomap &globalOctoMap, const sensor_msgs::PointCloud2 &globalPointCloud)
{
    /// drawing test path
    // Test pull request
    nav_msgs::Path myPath;
    myPath.header.stamp = ros::Time::now();
    myPath.header.frame_id = "/odom";

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

nav_msgs::Path Planner3D::planPath(const octomap_msgs::Octomap &globalOctoMap, const sensor_msgs::PointCloud2 &globalPointCloud)
{

    // search space information
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // TODO: define state checking callback
    //si->setStateValidityChecker(isStateValid);

    //set State Validity Checking Resolution (avoid going through the walls)
    si->setStateValidityCheckingResolution(0.001);

    // problem definition
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(*start.get(), *goal.get());

    // create planner
    auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));

    //configure planner
    planner->setRange(maxStepLength);
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);

    nav_msgs::Path plannedPath;

    if(solved)
    {
        //TODO: create extractPath function
        //plannedPath = extractPath(pdef.get());
    }
    

    return plannedPath;
}

void Planner3D::configure(void)
{
    dim = 3; //3D Problem
    maxStepLength = 0.1; // max step length

    //TODO: create correct bounds for all axes based on our enviroment
    /// create bounds for x axis
    coordXBound.reset(new ompl::base::RealVectorBounds(dim-1));
    coordXBound->setLow(-1.0);
    coordXBound->setHigh(13.0);

    /// create bounds for y axis
    coordYBound.reset(new ompl::base::RealVectorBounds(dim-1));
    coordYBound->setLow(-5.0);
    coordYBound->setHigh(5.0);

    /// create bounds for z axis
    coordZBound.reset(new ompl::base::RealVectorBounds(dim-1));
    coordZBound->setLow(0.0);
    coordZBound->setHigh(3.0);

    /// counstruct state space
    auto coordX(std::make_shared<ompl::base::RealVectorStateSpace>(dim-1));
    auto coordY(std::make_shared<ompl::base::RealVectorStateSpace>(dim-1));
    auto coordZ(std::make_shared<ompl::base::RealVectorStateSpace>(dim-1));
    space = coordX + coordY + coordZ;

    /// create bounds for all axes
    coordX->setBounds(*coordXBound.get());
    coordY->setBounds(*coordYBound.get());
    coordZ->setBounds(*coordZBound.get());


    // TODO: define proper start and goal positions based on our enviroment
    /// define the start position
    start.reset(new ompl::base::ScopedState<>(space));
    (*start.get())[0]=0.0;
    (*start.get())[1]=-2.5;
    (*start.get())[2]=0.1;

    //define the goal position
    goal.reset(new ompl::base::ScopedState<>(space));
    (*goal.get())[0]=12.0;
    (*goal.get())[1]=-4.0;
    (*goal.get())[2]=2.0;




}
}
