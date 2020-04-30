
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
    // TODO: WHOLE FUNCTION
    /// get current coordinnates of the robot
    const auto *coordinnates = state->as<ompl::base::CompoundState>()
            ->as<ompl::base::RealVectorStateSpace::StateType>(0);
    /// coordinnates->values[0] means x axis
    /// coordinnates->values[1] means y axis
    /// coordinnates->values[2] means z axis

    // comment lines below if you want to use octomap
    /// define an example obstacle
    if(coordinnates->values[0]<5.1 && coordinnates->values[0]>1.5) /// x axis
    {
        if(coordinnates->values[1]<2.5 && coordinnates->values[1]>0.4) /// y axis
        {
            if (coordinnates->values[2]<1.0 && coordinnates->values[2]>0.0) /// z axis
            {
                return false;
            }
        }
    }
    // comment lines above if you want to use octomap

    return true;
}


nav_msgs::Path Planner3D::extractPath(ompl::base::ProblemDefinition *pdef)
{
    nav_msgs::Path plannedPath;
    plannedPath.header.frame_id = "/odom";
    /// get the obtained path
    ompl::base::PathPtr path = pdef->getSolutionPath();
    /// print path to screen
    path->print(std::cout);
    /// convert to geometric path
    const auto*path_ = path.get()->as<ompl::geometric::PathGeometric>();
    /// iterate over each position
    for(unsigned int i=0; i<path_->getStateCount();++i)
    {
        /// get state
        const ompl::base::State* state = path_->getState(i);
        /// get coordinates of robot
        const auto *coordinates = state->as<ompl::base::CompoundState>()->
                as<ompl::base::RealVectorStateSpace::StateType>(0);
        /// fill in the ROS PoseStamped Structure
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordinates->values[0];
        poseMsg.pose.position.y = coordinates->values[1];
        poseMsg.pose.position.z = coordinates->values[2];
        poseMsg.pose.orientation.w = 1.0; // in the future this might also be calculated from coordinates, probably [3,4,5,6]
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id = "/odom";
        poseMsg.header.stamp = ros::Time::now();
        /// add poseStamped to the path
        plannedPath.poses.push_back(poseMsg);
    }
    return plannedPath;
}

nav_msgs::Path Planner3D::planPath(const octomap_msgs::Octomap &globalOctoMap, const sensor_msgs::PointCloud2 &globalPointCloud)
{

    /// planned Path
    nav_msgs::Path plannedPath;
    /// creating space information for the state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    //TODO: create isStateValid function that's using octomap
    ///Set the state validity checker
    si->setStateValidityChecker(isStateValid);
    /// create problem definition
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    /// set the start and goal states for the problem definiton
    pdef->setStartAndGoalStates(*start.get(),*goal.get());
    /// create instance of planner
    auto planner(std::make_shared<ompl::geometric::LazyPRMstar>(si));
    /// tell planner which problem we are intrested in solving
    planner->setProblemDefinition(pdef);
    /// setup the planner, after all settings for the space and planner are done
    planner->setup();
    /// problem status
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);
    /// check if problem is solved
    if (solved)
    {
        /// extract path from problem definition
        plannedPath = extractPath(pdef.get());
    }
    return plannedPath;

}

void Planner3D::configure(void)
{
    dim = 3; ///3D Problem
    maxStepLength = 0.1; /// max step length

    space.reset(new ompl::base::SE3StateSpace());

    bounds.reset(new ompl::base::RealVectorBounds(dim));

    // TODO: create proper bounds for each dimension
    /// Set the lower and higher bound for each dimension.
    /// x axis
    bounds->setLow(0, -5.0);
    bounds->setHigh(0, 15.0);
    /// y axis
    bounds->setLow(1,-5.0);
    bounds->setHigh(1,5.0);
    /// z axis
    bounds->setLow(2, 0.0);
    bounds->setHigh(2, 3.0);

    /// apply bounds to state space
    space->setBounds(*bounds.get());

    /// define starting state
    start.reset(new ompl::base::ScopedState<>(space));

    (*start.get())[0]=0.0; /// x
    (*start.get())[1]=0.0; /// y
    (*start.get())[2]=0.1; /// z
    (*start.get())[3]=0.0; /// qx
    (*start.get())[4]=0.0; /// qy
    (*start.get())[5]=0.0; /// qz
    (*start.get())[6]=1.0; /// qw

    /// define goal state
    goal.reset(new ompl::base::ScopedState<>(space));
    (*goal.get())[0]=5.3; /// x
    (*goal.get())[1]=2.6; /// y
    (*goal.get())[2]=1.5; /// z
    (*goal.get())[3]=0.0; /// qx
    (*goal.get())[4]=0.0; /// qy
    (*goal.get())[5]=0.0; /// qz
    (*goal.get())[6]=1.0; /// qw


}
}
