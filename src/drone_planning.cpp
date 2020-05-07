
#include "../include/drone_planning/drone_planning.hpp"

using namespace std;
using namespace ros;



namespace drone_planning{

octomap::OcTree* globalOctomapOcTree;
fcl::OcTree<double>* globalFCLOcTree;

/// drone's meshes
std::vector<fcl::Vec3f> droneMeshPoints;
std::vector<fcl::Triangle> droneMeshTriangles;




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

void loadRobotMesh(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles){


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
          fcl::Vec3f p(x, y, z);
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
    std::cout << "getDefaultOccupancy(): " << globalFCLOcTree->getDefaultOccupancy() << "\n"; /// example of getting to FCL:OcTree data

    //TODO: Use this meshes to check colission
    std::cout <<"meshPoints size " <<  droneMeshPoints.size() << "\n";
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


    template <typename S>
    void octomap_collision_test(S env_scale, std::size_t env_size, bool exhaustive, std::size_t num_max_contacts, bool use_mesh, bool use_mesh_octomap, double resolution)
    {
        // srand(1);
        std::vector<CollisionObject<S>*> env;
        if(use_mesh)
            test::generateEnvironmentsMesh(env, env_scale, env_size);
        else
            test::generateEnvironments(env, env_scale, env_size);

        OcTree<S>* tree = new OcTree<S>(std::shared_ptr<const octomap::OcTree>(test::generateOcTree(resolution)));
        CollisionObject<S> tree_obj((std::shared_ptr<CollisionGeometry<S>>(tree)));

        DynamicAABBTreeCollisionManager<S>* manager = new DynamicAABBTreeCollisionManager<S>();
        manager->registerObjects(env);
        manager->setup();

        DefaultCollisionData<S> cdata;
        if(exhaustive) cdata.request.num_max_contacts = 100000;
        else cdata.request.num_max_contacts = num_max_contacts;

        test::TStruct t1;
        test::Timer timer1;
        timer1.start();
        manager->octree_as_geometry_collide = false;
        manager->octree_as_geometry_distance = false;
        manager->collide(&tree_obj, &cdata, DefaultCollisionFunction);
        timer1.stop();
        t1.push_back(timer1.getElapsedTime());

        DefaultCollisionData<S> cdata3;
        if(exhaustive) cdata3.request.num_max_contacts = 100000;
        else cdata3.request.num_max_contacts = num_max_contacts;

        test::TStruct t3;
        test::Timer timer3;
        timer3.start();
        manager->octree_as_geometry_collide = true;
        manager->octree_as_geometry_distance = true;
        manager->collide(&tree_obj, &cdata3, DefaultCollisionFunction);
        timer3.stop();
        t3.push_back(timer3.getElapsedTime());

        test::TStruct t2;
        test::Timer timer2;
        timer2.start();
        std::vector<CollisionObject<S>*> boxes;
        if(use_mesh_octomap)
            test::generateBoxesFromOctomapMesh(boxes, *tree);
        else
            test::generateBoxesFromOctomap(boxes, *tree);
        timer2.stop();
        t2.push_back(timer2.getElapsedTime());

        timer2.start();
        DynamicAABBTreeCollisionManager<S>* manager2 = new DynamicAABBTreeCollisionManager<S>();
        manager2->registerObjects(boxes);
        manager2->setup();
        timer2.stop();
        t2.push_back(timer2.getElapsedTime());


        DefaultCollisionData<S> cdata2;
        if(exhaustive) cdata2.request.num_max_contacts = 100000;
        else cdata2.request.num_max_contacts = num_max_contacts;

        timer2.start();
        manager->collide(manager2, &cdata2, DefaultCollisionFunction);
        timer2.stop();
        t2.push_back(timer2.getElapsedTime());

        std::cout << cdata.result.numContacts() << " " << cdata3.result.numContacts() << " " << cdata2.result.numContacts() << std::endl;
        if(exhaustive)
        {
            if(use_mesh) EXPECT_TRUE((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0));
            else EXPECT_TRUE(cdata.result.numContacts() == cdata2.result.numContacts());
        }
        else
        {
            if(use_mesh) EXPECT_TRUE((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0));
            else EXPECT_TRUE((cdata.result.numContacts() > 0) >= (cdata2.result.numContacts() > 0)); // because AABB<S> return collision when two boxes contact
        }

        delete manager;
        delete manager2;
        for(size_t i = 0; i < boxes.size(); ++i)
            delete boxes[i];

        if(exhaustive) std::cout << "exhaustive collision" << std::endl;
        else std::cout << "non exhaustive collision" << std::endl;
        std::cout << "1) octomap overall time: " << t1.overall_time << std::endl;
        std::cout << "1') octomap overall time (as geometry): " << t3.overall_time << std::endl;
        std::cout << "2) boxes overall time: " << t2.overall_time << std::endl;
        std::cout << "  a) to boxes: " << t2.records[0] << std::endl;
        std::cout << "  b) structure init: " << t2.records[1] << std::endl;
        std::cout << "  c) collision: " << t2.records[2] << std::endl;
        std::cout << "Note: octomap may need more collides when using mesh, because octomap collision uses box primitive inside" << std::endl;
    }


    template <typename S>
    void test_octomap_collision_mesh()
    {
#ifdef NDEBUG
        octomap_collision_test<S>(200, 100, false, 10, true, true);
  octomap_collision_test<S>(200, 1000, false, 10, true, true);
  octomap_collision_test<S>(200, 100, true, 1, true, true);
  octomap_collision_test<S>(200, 1000, true, 1, true, true);
#else
        octomap_collision_test<S>(200, 4, false, 1, true, true, 1.0);
        octomap_collision_test<S>(200, 4, true, 1, true, true, 1.0);
#endif
    }

    GTEST_TEST(FCL_OCTOMAP, test_octomap_collision_mesh)
{
//  test_octomap_collision_mesh<float>();
    test_octomap_collision_mesh<double>();
}


void Planner3D::configure(void)
{
    dim = 3; ///3D Problem
    maxStepLength = 0.1; /// max step length

    /// relative path to robot's mesh
    string meshPath =  ros::package::getPath("drone_planning") + "/meshes/quadrotor/quadrotor_base.obj"; /// tried .dae .stl

    /// loading drone's mesh
    loadRobotMesh(meshPath.c_str(), droneMeshPoints, droneMeshTriangles);
    std::cout <<"meshPoints size " <<  droneMeshPoints.size() << "\n";
    //droneMeshPoints.
    std::cout <<"meshTriangles size " <<  droneMeshTriangles.size() << "\n";

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
