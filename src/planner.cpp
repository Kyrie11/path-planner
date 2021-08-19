#include "../include/planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner(ros::NodeHandle& nh) {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  grid = boost::make_shared<nav_msgs::OccupancyGrid>();

  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  
  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/odom_visualization/pose", 1, &Planner::setStart, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }
  std::cout << "I am seeing the map..." << std::endl;
  grid = map;
  mapHeight = map->info.height;
  mapWidth = map->info.width;
  originX = map->info.origin.position.x;
  originY = map->info.origin.position.y;
  std::cout<<"originX is:"<<originX<<",originY is:"<<originY<<std::endl;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  
  // std::cout<<"\n height:"<<height<<",width:"<<width<<std::endl;
  bool** binMap;
  binMap = new bool*[mapWidth];
  std::cout<<"the map's width is:"<<mapWidth<<", height is:"<<mapHeight<<std::endl;
  for (int x = 0; x < mapWidth; x++) { binMap[x] = new bool[mapHeight]; }

  for (int x = 0; x <  mapWidth; ++x) {
    for (int y = 0; y < mapHeight; ++y) {
      binMap[x][y] = map->data[y * mapWidth + x] > 0 ? true : false;
      // std::cout<<binMap[x][y]<<" ";   
    }
    // std::cout<<std::endl;
  }

  voronoiDiagram.initializeMap(mapWidth, mapHeight, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize("/home/coola/result.pgm");
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.position.x = transform.getOrigin().x();
    start.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.orientation);

    if (mapHeight+originY >= start.pose.position.y && start.pose.position.y >= originY-1 &&
        mapWidth+originX >= start.pose.position.x && start.pose.position.x >= originX-1) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    // plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseStamped::ConstPtr& initial) {
  // std::cout<<"initial:"<<*initial;
  float x = initial->pose.position.x / Constants::cellSize;
  float y = initial->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.position;
  startN.pose.orientation = initial->pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  // std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  // std::cout<<"height:"<<mapHeight<<",width:"<<mapWidth<<std::endl;
  // std::cout<<"originX:"<<originX<<",originY:"<<originY<<std::endl;
  if (mapHeight + originY >= y && y >= originY && mapWidth + originX >= x && x >= originX) {
    validStart = true;
    start = *initial;
    // start.pose.position.x -= originX;
    // start.pose.position.y -= originY;

    // if (Constants::manual) { plan();}

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (mapHeight + originY >= y && y >= originY && mapWidth + originX >= x && x >= originX) {
    validGoal = true;
    goal = *end;
    // goal.pose.poistion.x -= originX;
    // goal.pose.position.y -= originY;
    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER

    int depth = Constants::headings;
    int length = mapWidth * mapHeight * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[mapWidth * mapHeight]();

    // ________________________
    // retrieving goal position
    float x = (goal.pose.position.x - originX) / Constants::cellSize;
    float y = (goal.pose.position.y - originY) / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = (start.pose.position.x  - originX) / Constants::cellSize;
    y = (start.pose.position.y  - originY) / Constants::cellSize;
    t = tf::getYaw(start.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, mapWidth, mapHeight, configurationSpace, dubinsLookup, visualization);
    // std::cout<<"\n nSolution:"<<nSolution<<std::endl;
    // TRACE THE PATH
    smoother.tracePath(nSolution, originX, originY);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram, originX, originY);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;



    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, mapWidth, mapHeight, depth);
    visualization.publishNode2DCosts(nodes2D, mapWidth, mapHeight);



    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
