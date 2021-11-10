#include <cmath>
#include <unordered_map>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "visualization/visualization.h"

#include "planner.h"
#include "navigation.h"
#include "simple_queue.h"

using namespace std;
using namespace math_util;
using namespace ros_helpers;
using amrl_msgs::VisualizationMsg;

namespace
{
  ros::Publisher viz_pub_;
  VisualizationMsg local_viz_msg_;
  VisualizationMsg global_viz_msg_;
  // Epsilon value for handling limited numerical precision.
  const float kEpsilon = 1e-3;
} //namespace

namespace planner {

Planner::Planner(): global_goal_mloc_(0,0),
                    global_goal_mangle_(0),
                    global_goal_set_(false)
{
  
}

const vector<Vector2f>& Planner::GetPath() {
  return path;
}

void Planner::SetGlobalGoal(const Vector2f &loc, float angle) {
  global_goal_mloc_ = loc;
  global_goal_mangle_ = angle;
  global_goal_set_ = true;
}

float Planner::getScore(const Vector2f& loc, const Vector2f& prev_loc, float prev_cost) {
  // if (!global_goal_set_) {return;}
  
  float cost = prev_cost + (prev_loc - loc).norm(); // TODO: check this
  float heuristic = (global_goal_mloc_ - loc).norm();
  return cost + heuristic;
}

void Planner::GetGlobalPlan(const Vector2f& robot_mloc, float robot_mangle) {
  if (!global_goal_set_) {return;}

  path.clear();

  SimpleQueue<Vector2f, float> frontier;
  frontier.Push(robot_mloc, 0);
  
  // keep track of visited search states and their parents
  // key: child, value: parent
  unordered_map<Vector2f, Vector2f> visited; // TODO: add hash

  // parent = {}, parent[start] = Null
// cost = {}
// cost[start] = 0             // Cost from start

// while (!frontier.Empty()){
// current = frontier.get()  // Get by priority!
//   if current == goal:
//     break
//   for next in neighbors(current):
//     new_cost = cost[current] + EdgeCost[current, next]
//     if next not in cost or new_cost < cost[next]:
//       cost[next] = new_cost  // Insertion or edit
//       frontier.put(next, new_cost + heuristic(next))
//       parent[next] = current
// }


}

} // namespace planner
