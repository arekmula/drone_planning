
#include "../include/drone_planning/drone_planning.hpp"

using namespace std;
using namespace ros;



namespace drone_planning{

octomap::OcTree* globalOctomapOcTree;
fcl::OcTree<double>* globalFCLOcTree;
std::shared_ptr<fcl::CollisionGeometry<double>> globalCollisionGeometryOcTree;

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
    // TODO: WHOLE FUNCTION
    /// get current coordinnates of the robot
    const auto *translation = state->as<ompl::base::CompoundState>()
            ->as<ompl::base::RealVectorStateSpace::StateType>(0); /// translation
    const auto *quaternion = state->as<ompl::base::CompoundState>()
            ->as<ompl::base::SO3StateSpace::StateType>(1); /// quaternion

    // comment lines below if you don't want to use example obstacle
    // its an example obstacle
    /// define an example obstacle
    if(translation->values[0]<5.1 && translation->values[0]>1.5) /// x axis
    {
        if(translation->values[1]<2.5 && translation->values[1]>0.4) /// y axis
        {
            if (translation->values[2]<1.0 && translation->values[2]>0.0) /// z axis
            {
                return false;
            }
        }
    }
    // comment lines above if you don't want to use example obstacle

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

nav_msgs::Path Planner3D::planPath(const octomap_msgs::Octomap& octomapMsg)
{

    octomap::AbstractOcTree* my_tree = octomap_msgs::fullMsgToMap(octomapMsg); /// octomap message to AbstractOcTree
    globalOctomapOcTree = dynamic_cast<octomap::OcTree*>(my_tree); /// casting AbstractOcTree to OcTree

    /// examples of getting to octomap::OcTree data
    double xmax,ymax,zmax,xmin,ymin,zmin;
    globalOctomapOcTree->getMetricMax(xmax, ymax, zmax);
    globalOctomapOcTree->getMetricMin(xmin,ymin,zmin);
    std::cout <<"Octree resolution: " << globalOctomapOcTree->getResolution() << "\n";
    std::cout <<"Octree maxes :" << xmax <<" "<< ymax <<" " << zmax <<"\n";
    std::cout <<"Octree mins :" << xmin <<" "<< ymin <<" " << zmin <<"\n";


    //TODO: Use this OcTree below to check collision
    /// converting from octomap::OcTree to fcl::OcTree
    globalFCLOcTree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(globalOctomapOcTree));
    std::cout << globalFCLOcTree->getDefaultOccupancy() << "\n"; /// example of getting to FCL:OcTree data

    /// create CollisionGeomeetry from OcTree
    globalCollisionGeometryOcTree = std::shared_ptr<fcl::CollisionGeometry<double>>(globalFCLOcTree);

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

    // this version below works
    fcl::CollisionObjectd* obj = new fcl::CollisionObjectd(globalCollisionGeometryOcTree);
    // this version below should work, but it doesn't
//    fcl::CollisionObjectd* obj = new fcl::CollisionObjectd(globalCollisionGeometryOcTree, poseOctomap);



    std::cout <<"meshPoints size " <<  droneMeshVertices.size() << "\n";
    std::cout <<"meshTriangles size " <<  droneMeshTriangles.size() << "\n";

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

    /// relative path to robot's mesh
    string meshPath =  ros::package::getPath("drone_planning") + "/meshes/quadrotor/quadrotor_base.obj"; /// tried .dae .stl

    /// loading drone's mesh
    loadRobotMesh(meshPath.c_str(), droneMeshVertices, droneMeshTriangles);
    std::cout <<"meshPoints size " <<  droneMeshVertices.size() << "\n";
    std::cout <<"meshTriangles size " <<  droneMeshTriangles.size() << "\n";

    /// add the mesh data into the BVHModel structure
    geom = std::make_shared<Model>();
    geom->beginModel();
    geom->addSubModel(droneMeshVertices, droneMeshTriangles);
    geom->endModel();


    space.reset(new ompl::base::SE3StateSpace());

    bounds.reset(new ompl::base::RealVectorBounds(dim));


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
