#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <cmath>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"

namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;

/*!
 * \brief A class that encompasses the functions central to the search.
 */
class Algorithm {
 public:
  /// The deault constructor
  Algorithm();
  Algorithm(ros::NodeHandle& nh);
/*   {
  ros::NodeHandle& n;
  odomSub = n.subscribe("/camera/odom/sample", 10, &Algorithm::odomCallback, this);
  pubPositionCmd = n.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
  
}*/

  // HYBRID A* ALGORITHM
  /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
     \param width the width of the grid in number of cells
     \param height the height of the grid in number of cells
     \param configurationSpace the lookup of configurations and their spatial occupancy enumeration
     \param dubinsLookup the lookup of analytical solutions (Dubin's paths)
     \param visualization the visualization object publishing the search to RViz
     \return the pointer to the node satisfying the goal condition
  */
   Node3D* hybridAStar(Node3D& start,
                             const Node3D& goal,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                             Visualize& visualization);


  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
    Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);
    nav_msgs::Odometry odom;
    ros::NodeHandle n;
    double turn_radius;
};
}
#endif // ALGORITHM_H
