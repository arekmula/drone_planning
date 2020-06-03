#include "../include/drone_planning/drone_planning.hpp"

using namespace std;
using namespace ros;



namespace drone_planning{

/// variables needed to convert delivered octomap to collision object of this octomap
octomap::OcTree* globalOctomapOcTree;
fcl::OcTree<float>* globalFCLOcTree;
std::shared_ptr<fcl::CollisionGeometry<float>> globalCollisionGeometryOcTree; /// collision geometry of octoamp
fcl::CollisionObjectf* globalCollisionObjectOcTree; /// collision object of octomap

/// drone's meshes
std::vector<fcl::Vector3f> droneMeshVertices;
std::vector<fcl::Triangle> droneMeshTriangles;

/// BVHModel is a template class for mesh geometry, for default OBBRSS template is used
typedef fcl::BVHModel<fcl::OBBRSSf> Model;
std::shared_ptr<Model> geom;




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
    /// get current coordinnates of the robot
    const auto *translation = state->as<ompl::base::CompoundState>()
            ->as<ompl::base::RealVectorStateSpace::StateType>(0); /// translation
    const auto *quaternion = state->as<ompl::base::CompoundState>()
            ->as<ompl::base::SO3StateSpace::StateType>(1); /// quaternion

    /// R and T are the rotation matrix and translation vector of drone
    fcl::Matrix3f R;
    fcl::Vector3f T;
    /// save current drone translation based on state to FCL translation vector
    T(0) = translation->values[0]; /// x
    T(1) = translation->values[1]; /// y
    T(2) = translation->values[2]; /// z
    /// save current drone rotation in quaternion to FCL quaternion
    fcl::Quaternionf q;
    q.x() = quaternion->x;
    q.y() = quaternion->y;
    q.z() = quaternion->z;
    q.w() = quaternion->w;
    /// X, Y, Z axes angle limit
    if(quaternion->x > 0.348 || quaternion->x < -0.348)
    {
        return false;
    }
    if(quaternion->y > 0.348 || quaternion->y < -0.348)
    {
        return false;
    }
    /// convert quaternion to rotation Matrix
    R = q.normalized().toRotationMatrix();
    /// Transform is configured according to R and T
    fcl::Transform3f pose = fcl::Transform3f::Identity(); /// pose of drone
    pose.linear()=R;
    pose.translation()=T;
    /// geom and pose(tf in tutorial, https://github.com/flexible-collision-library/fcl)
    /// are the geometry and the transform of the object
    // (?) not sure about this pose variable (?)
    fcl::CollisionObjectf* drone = new fcl::CollisionObjectf(geom,pose); /// collision object of drone
    /// create request and result variables
    fcl::CollisionRequest<float> request;
    fcl::CollisionResult<float> result;
    /// check if there is collision between drone and enviroment
    int collides = fcl::collide(drone, globalCollisionObjectOcTree, request, result);
    /// if there's any collision return state as invalid
    if (collides>0)
        return false;
    return true;

}

void loadRobotMesh(const char* filename, std::vector<fcl::Vector3f>& points, std::vector<fcl::Triangle>& triangles){

    FILE* file = fopen(filename, "rb");
    if(!file)
    {
        std::cout << "file not exist" << "\n";
        return;
    }
    else{
        std::cout << "Started reading mesh data" << "\n";
    }

    bool has_normal = false;
    bool has_texture = false;
    char line_buffer[2000];
    while(fgets(line_buffer, 2000, file))
    {
        char* first_token = strtok(line_buffer, "\r\n\t ");
        if(!first_token || first_token[0] == '#' || first_token[0] == 0)
            continue;

        switch(first_token[0])
        {
        case 'v':
        {
            if(first_token[1] == 'n')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_normal = true;
            }
            else if(first_token[1] == 't')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_texture = true;
            }
            else
            {
                fcl::FCL_REAL x = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::FCL_REAL y = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::FCL_REAL z = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::Vector3f p(x, y, z);
                points.push_back(p);
            }
        }
            break;
        case 'f':
        {
            fcl::Triangle tri;
            char* data[30];
            int n = 0;
            while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
            {
                if(strlen(data[n]))
                    n++;
            }

            for(int t = 0; t < (n - 2); ++t)
            {
                if((!has_texture) && (!has_normal))
                {
                    tri[0] = atoi(data[0]) - 1;
                    tri[1] = atoi(data[1]) - 1;
                    tri[2] = atoi(data[2]) - 1;
                }
                else
                {
                    const char *v1;
                    for(int i = 0; i < 3; i++)
                    {
                        // vertex ID
                        if(i == 0)
                            v1 = data[0];
                        else
                            v1 = data[t + i];

                        tri[i] = atoi(v1) - 1;
                    }
                }
                triangles.push_back(tri);
            }
        }
        }
    }
}

bool isRandomStateValid(float xPos, float yPos, float zPos)
{
    /// R and T are the rotation matrix and translation vector of drone
    fcl::Matrix3f R;
    fcl::Vector3f T;
    /// save current drone translation based on state to FCL translation vector
    T(0) = xPos; /// x
    T(1) = yPos; /// y
    T(2) = zPos; /// z
    /// save current drone rotation in quaternion to FCL quaternion
    fcl::Quaternionf q;
    q.x() = 0.0;
    q.y() = 0.0;
    q.z() = 0.0;
    q.w() = 1.0;
    /// convert quaternion to rotation Matrix
    R = q.normalized().toRotationMatrix();
    /// Transform is configured according to R and T
    fcl::Transform3f pose = fcl::Transform3f::Identity(); /// pose of drone
    pose.linear()=R;
    pose.translation()=T;
    /// geom and pose(tf in tutorial, https://github.com/flexible-collision-library/fcl)
    /// are the geometry and the transform of the object
    fcl::CollisionObjectf* drone = new fcl::CollisionObjectf(geom,pose); /// collision object of drone
    /// create request and result variables
    fcl::CollisionRequest<float> request;
    fcl::CollisionResult<float> result;
    /// check if there is collision between drone and enviroment
    int collides = fcl::collide(drone, globalCollisionObjectOcTree, request, result);
    /// if there's any collision return state as invalid
    if (collides>0)
    {
        return false;
    }
    std::cout << "Found new random goal state" << "\n";
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
        const auto *rotation = state->as<ompl::base::CompoundState>()->
                as<ompl::base::SO3StateSpace::StateType>(1);
        /// fill in the ROS PoseStamped Structure
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordinates->values[0];
        poseMsg.pose.position.y = coordinates->values[1];
        poseMsg.pose.position.z = coordinates->values[2];
        poseMsg.pose.orientation.w = rotation->w;
        poseMsg.pose.orientation.x = rotation->x;
        poseMsg.pose.orientation.y = rotation->y;
        poseMsg.pose.orientation.z = rotation->z;
        poseMsg.header.frame_id = "/odom";
        poseMsg.header.stamp = ros::Time::now();
        /// add poseStamped to the path
        plannedPath.poses.push_back(poseMsg);
    }
    return plannedPath;
}

nav_msgs::Path Planner3D::planPath(const octomap_msgs::Octomap& octomapMsg)
{
    /// converting octomap message to AbstractOcTree
    octomap::AbstractOcTree* my_tree = octomap_msgs::fullMsgToMap(octomapMsg);
    /// casting AbstractOcTree to OcTree
    globalOctomapOcTree = dynamic_cast<octomap::OcTree*>(my_tree);
    /// converting from octomap::OcTree to fcl::OcTree
    globalFCLOcTree = new fcl::OcTree<float>(std::shared_ptr<const octomap::OcTree>(globalOctomapOcTree));
    std::cout << globalFCLOcTree->getDefaultOccupancy() << "\n"; /// example of getting to FCL:OcTree data
    /// create CollisionGeomeetry from OcTree
    globalCollisionGeometryOcTree = std::shared_ptr<fcl::CollisionGeometry<float>>(globalFCLOcTree);

    fcl::Matrix3f Roctomap; /// rotation matrix of octomap
    fcl::Vector3f Toctomap; /// translation vector of octomap
    /// get translation of octomap
    Toctomap(0)=globalOctomapOcTree->getBBXCenter().x();
    Toctomap(1)=globalOctomapOcTree->getBBXCenter().y();
    Toctomap(2)=globalOctomapOcTree->getBBXCenter().z();
    /// get rotation of octomap as euler
    fcl::AngleAxisf rollAngle(globalOctomapOcTree->getBBXCenter().roll(),fcl::Vector3f::UnitZ());
    fcl::AngleAxisf pitchAngle(globalOctomapOcTree->getBBXCenter().pitch(),fcl::Vector3f::UnitX());
    fcl::AngleAxisf yawAngle(globalOctomapOcTree->getBBXCenter().yaw(),fcl::Vector3f::UnitY());
    /// convert it to quaternion
    fcl::Quaternion<float> q = rollAngle*pitchAngle*yawAngle;
    /// convert it to rotation matrix
    Roctomap = q.matrix();
    /// get transform of octomap
    fcl::Transform3f poseOctomap = fcl::Transform3f::Identity();
    poseOctomap.linear()=Roctomap;
    poseOctomap.translation()=Toctomap;
    /// create collision object of octomap
    /// it is used in isStateValid to check collision with drone
    globalCollisionObjectOcTree = new fcl::CollisionObjectf(globalCollisionGeometryOcTree, poseOctomap);

    /// planned Path
    nav_msgs::Path plannedPath;
    /// creating space information for the state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    ///Set the state validity checker
    si->setStateValidityChecker(isStateValid);
    /// create problem definition
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    /// set the start and goal states for the problem definiton
    pdef->setStartAndGoalStates(*start.get(),*goal.get());
    std::cout << "Start used by planner: " << *start.get() << "\n";
    std::cout << "Goal used by planner: " << *goal.get() << "\n";
    /// create instance of planner
    auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
    /// tell planner which problem we are intrested in solving
    planner->setProblemDefinition(pdef);
    /// setup the planner, after all settings for the space and planner are done
    planner->setup();
    /// problem status
    std::cout << "Searching for path for next 10 seconds" << "\n";
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(10.0);
    /// check if planner found exact solution
    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
        /// extract path from problem definition
        plannedPath = extractPath(pdef.get());
        initialMove=false; ///
        /// save current start position to pass it to visualization
        saveStartState();
        /// randomize
        randomizeNewGoalState();
    }
    return plannedPath;

}

void Planner3D::saveStartState(void)
{
    xStart = (*start.get())[0];
    yStart = (*start.get())[1];
    zStart = (*start.get())[2];

    std::cout << "Saved start state: " << xStart << " " << yStart << " " << zStart << "\n";
}



void Planner3D::randomizeNewGoalState(void)
{
//    start.reset(new ompl::base::ScopedState<>(space));
    if (!initialMove)
    {
        (*start.get())[0]=xGoal; /// x
        (*start.get())[1]=yGoal; /// y
        (*start.get())[2]=zGoal; /// z
        (*start.get())[3]=0.0; /// qx
        (*start.get())[4]=0.0; /// qy
        (*start.get())[5]=0.0; /// qz
        (*start.get())[6]=1.0; /// qw

        xGoal=0, yGoal=0, zGoal=0;
        while (!isRandomStateValid(xGoal, yGoal, zGoal))
        {
//            goal.reset(new ompl::base::ScopedState<>(space));
            goal.get()->random();
            (*goal.get())[3]=0.0; /// qx
            (*goal.get())[4]=0.0; /// qy
            (*goal.get())[5]=0.0; /// qz
            (*goal.get())[6]=1.0; /// qw
            xGoal = (*goal.get())[0];
            yGoal = (*goal.get())[1];
            zGoal = (*goal.get())[2];
        }
    }
    /// TODO: add a possibility to quickly change between random goal positions and set goal positions
    std::cout << "Start state copied from last goal: " << (*start.get())[0] << " " << (*start.get())[1]<<
                 " " << (*start.get())[2] << "\n";
    std::cout << "Randomized goal state: " << (*goal.get())[0] << " " << (*goal.get())[1]<<
                 " " << (*goal.get())[2] << "\n";

}



void Planner3D::getStartPosition(float &xPos, float &yPos, float &zPos)
{
    xPos = xStart;
    yPos = yStart;
    zPos = zStart;
}

void Planner3D::configure(void)
{
    dim = 3; ///3D Problem

    /// relative path to robot's mesh
    string meshPath =  ros::package::getPath("drone_planning") + "/meshes/quadrotor/quadrotor_2.obj"; /// tried .dae .stl

    /// loading drone's mesh
    loadRobotMesh(meshPath.c_str(), droneMeshVertices, droneMeshTriangles);

    /// add the mesh data into the BVHModel structure
    geom = std::make_shared<Model>();
    geom->beginModel();
    geom->addSubModel(droneMeshVertices, droneMeshTriangles);
    geom->endModel();

    /// initialize SE3 Space and Bounds
    space.reset(new ompl::base::SE3StateSpace());
    bounds.reset(new ompl::base::RealVectorBounds(dim));

    srand(time(NULL));

    /// Set the lower and higher bound for each dimension.
    /// Based on data from globalOcTree
    /// x axis
    bounds->setLow(0, -7.6);
    bounds->setHigh(0, 7.85);
    /// y axis

    bounds->setLow(1,-7.9);
    bounds->setHigh(1,5.4);

    /// z axis
    bounds->setLow(2, 0.0);
    bounds->setHigh(2, 2.35);

    /// apply bounds to state space
    space->setBounds(*bounds.get());

    /// set initial start and goal states
    start.reset(new ompl::base::ScopedState<>(space));
    (*start.get())[0]=-1.0; /// x
    (*start.get())[1]=0.5; /// y
    (*start.get())[2]=0.3; /// z
    (*start.get())[3]=0.0; /// qx
    (*start.get())[4]=0.0; /// qy
    (*start.get())[5]=0.0; /// qz
    (*start.get())[6]=1.0; /// qw

    goal.reset(new ompl::base::ScopedState<>(space));
    (*goal.get())[0]=5.3; /// x
    (*goal.get())[1]=2.6; /// y
    (*goal.get())[2]=2.0; /// z
    (*goal.get())[3]=0.0; /// qx
    (*goal.get())[4]=0.0; /// qy
    (*goal.get())[5]=0.0; /// qz
    (*goal.get())[6]=1.0; /// qw

    /// save goal position, as we assume that rotation in goal state is always (0, 0, 0, 1)
    xGoal = (*goal.get())[0];
    yGoal = (*goal.get())[1];
    zGoal = (*goal.get())[2];

}
}
