
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

    //get the obtained path
    ompl::base::PathPtr path = pdef->getSolutionPath();

    //print the path to screen
    std::cout << "Printing path 1" << "\n";
    path->print(std::cout);
    std::cout << "Printing path 2" << "\n";

    //convert to geometric path
    const auto *path_ = path.get()->as<ompl::geometric::PathGeometric>();
    std::cout << "Printing geometric path " << "\n";
    path_->print(std::cout);

//    const std::vector<ompl::base::State*> &states = path_->getStates();
//    ompl::base::State *state;

//    for(size_t i=0; i<states.size();++i)
//    {
//        state = states[i]->as<ompl::base::State>();
////        const auto *coordX = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
////        const auto *coordY = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
////        const auto *coordZ = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];


    //}

    // iterate over each position
   for(unsigned int i=0; i<path_->getStateCount(); ++i)
    {
        //get state
        const ompl::base::State* state = path_->getState(i);


        // get coords of the robot
        double coordX = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
        double coordY = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
        double coordZ = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
        // fill in the ROS PoseStamped Structure
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordX;
        poseMsg.pose.position.y = coordY;
        poseMsg.pose.position.z = coordZ;
        poseMsg.pose.orientation.w = 1.0;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id = "/odom";
        poseMsg.header.stamp = ros::Time::now();
        std::cout <<"Position: " << poseMsg.pose.position << "\n";
        plannedPath.poses.push_back(poseMsg);
    }
    return plannedPath;
}

nav_msgs::Path Planner3D::planPath(const octomap_msgs::Octomap &globalOctoMap, const sensor_msgs::PointCloud2 &globalPointCloud)
{
    /*
    // search space information
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // TODO: define state checking callback
    si->setStateValidityChecker(isStateValid);


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
        plannedPath = extractPath(pdef.get());
    }
    */

    nav_msgs::Path plannedPath;
    auto space(std::make_shared<ompl::base::SE3StateSpace>());

    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    si->setStateValidityChecker(isStateValid);


    /// define the start position
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setZ(0.0);
    start->rotation().setIdentity();

    /// define goal state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
    goal->setX(1.0);
    goal->setY(1.5);
    goal->setZ(1.0);
    goal->rotation().setIdentity();


    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);

    if(solved)
    {
        std::cout<< "Problem solved" << "\n";
        plannedPath = extractPath(pdef.get());
    }

    return plannedPath;
}

void Planner3D::configure(void)
{
    dim = 3; //3D Problem
    maxStepLength = 0.1; // max step length

/*
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
    coordZBound->setHigh(16.0);

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
    (*start.get())[0]=3.0;
    (*start.get())[1]=2.0;
    (*start.get())[2]=1.0;

    //define the goal position
    goal.reset(new ompl::base::ScopedState<>(space));
    (*goal.get())[0]=3.1;
    (*goal.get())[1]=2.1;
    (*goal.get())[2]=1.1;
*/
}
}
