
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

bool isStateValid(const ompl::base::State *state)
{
    // get coords of the robot
    const auto *coordX = state->as<ompl::base::CompoundState>()
            ->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const auto *coordY = state->as<ompl::base::CompoundState>()
                ->as<ompl::base::RealVectorStateSpace::StateType>(1);
    const auto *coordZ = state->as<ompl::base::CompoundState>()
            ->as<ompl::base::RealVectorStateSpace::StateType>(2);

    //! Comment this part of code if you'd like to use octomap
     //define the obstacle
//    if (coordX->values[0]<5.1&&coordX->values[0]>5.0){
//        if (coordY->values[0]<4.0&&coordY->values[0]>-5.0){
//            return false;
//        }
//    }
    //! Comment this part of code if you'd like to use octomap

    return true;
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

nav_msgs::Path Planner3D::extractPath(ompl::base::ProblemDefinition *pdef)
{
    nav_msgs::Path plannedPath;
    plannedPath.header.frame_id = "/odom";

    ompl::base::PathPtr path = pdef->getSolutionPath();

    path->print(std::cout);

    const auto*path_ = path.get()->as<ompl::geometric::PathGeometric>();

    for(unsigned int i=0; i<path_->getStateCount();++i)
    {
        const ompl::base::State* state = path_->getState(i);

        const auto *coordinates = state->as<ompl::base::CompoundState>()->
                as<ompl::base::RealVectorStateSpace::StateType>(0);

        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordinates->values[0];
        poseMsg.pose.position.y = coordinates->values[1];
        poseMsg.pose.position.z = coordinates->values[2];
        poseMsg.pose.orientation.w = 1.0;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id = "/odom";
        poseMsg.header.stamp = ros::Time::now();
        plannedPath.poses.push_back(poseMsg);
    }

    return plannedPath;
}

nav_msgs::Path Planner3D::planPath(const octomap_msgs::Octomap &globalOctoMap, const sensor_msgs::PointCloud2 &globalPointCloud)
{


    nav_msgs::Path plannedPath;
    auto space(std::make_shared<ompl::base::SE3StateSpace>());

    ompl::base::RealVectorBounds bounds(3);

    // Set the lower and higher bound of a dimension to a specific value.
    bounds.setLow(0, -5.0); // x axis
    bounds.setHigh(0, 15.0);

    bounds.setLow(1,-5.0); // y axis
    bounds.setHigh(1,5.0);

    bounds.setLow(2, 0.0); // z axis
    bounds.setHigh(2, 3.0);

    std::cout<< bounds.getVolume() << "\n";  //get volume of space

    space->setBounds(bounds);

    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    si->setStateValidityChecker(isStateValid);


    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setZ(0.1);
    start->rotation().setIdentity();

    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(space);
    goal->setX(13.0);
    goal->setY(2.5);
    goal->setZ(2.0);
    goal->rotation().setIdentity();

    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    pdef->setStartAndGoalStates(start,goal);

    auto planner(std::make_shared<ompl::geometric::LazyPRMstar>(si));
    planner->setProblemDefinition(pdef);

    planner->setup();

    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);

    if (solved)
    {
        plannedPath = extractPath(pdef.get());
    }


    return plannedPath;

}

void Planner3D::configure(void)
{
    dim = 3; //3D Problem
    maxStepLength = 0.1; // max step length

}
}
