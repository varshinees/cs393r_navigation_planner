#include <cmath>
#include <unordered_set>
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
#include "vector_map/vector_map.h"
#include "shared/math/geometry.h"

#include "planner.h"
#include "navigation.h"
#include "simple_queue.h"

using namespace std;
using namespace math_util;
using namespace ros_helpers;
using namespace Eigen;
using amrl_msgs::VisualizationMsg;
using geometry::line2f;

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
                    global_goal_set_(false),
                    lattices_({Vector2f(GRID_SIZE, 0), Vector2f(-GRID_SIZE, 0),
                               Vector2f(0, GRID_SIZE), Vector2f(0, -GRID_SIZE),
                               Vector2f(GRID_SIZE, GRID_SIZE), Vector2f(-GRID_SIZE, -GRID_SIZE),
                               Vector2f(GRID_SIZE, -GRID_SIZE), Vector2f(-GRID_SIZE, GRID_SIZE)})
{}

void Planner::SetMap(const string &map_file) {
  map_.Load(map_file);
}

const vector<Vector2f>& Planner::GetPath() {
  return path;
}

void Planner::SetGlobalGoal(const Vector2f &loc, float angle) {
  global_goal_mloc_ = loc;
  global_goal_mangle_ = angle;
  global_goal_set_ = true;
}

float Planner::GetScore_(const Vector2f& loc, const Vector2f& prev_loc, float prev_cost) {  
  float cost = prev_cost + (prev_loc - loc).norm(); // TODO: check this
  float heuristic = (global_goal_mloc_ - loc).norm();
  return cost + heuristic;
}

void Planner::Neighbors_(const Vector2f& loc, vector<Vector2f>* neighbors) {
  Vector2f intersection_point(0,0);
  for (Vector2f lattice : lattices_) {
    Vector2f next = loc + lattice;
    line2f line(loc.x(), loc.y(), next.x(), next.y());
    bool intersects = false;
    for (line2f map_line : map_.lines) {
      intersects = map_line.Intersection(line, &intersection_point);
      if (intersects) { break; }
    }
    if (!intersects) {
      neighbors->push_back(next);
    }
  }
}

// checks if current location is close enough to the goal location
bool Planner::AtGoal(const Vector2f& robot_mloc) {
  return (robot_mloc - global_goal_mloc_).norm() < GRID_SIZE;
}

void Planner::GetGlobalPlan(const Vector2f& robot_mloc, float robot_mangle) {
  if (!global_goal_set_) {return;}

  path.clear();

  // priority queue stores <SearchState, score>
  SimpleQueue<SearchState, float> frontier;
  frontier.Push(SearchState{curr_loc: robot_mloc, cost: 0, parent: nullptr}, 0);
  
  // keep track of visited search states
  unordered_set<Vector2f, Vector2fHash> visited;
  // keep track of the parents of each search states
  // key: child, value: parent
  // unordered_map<Vector2f, Vector2f, Vector2fHash> parents;

  // parent = {}, parent[start] = Null
  // cost = {}
  // cost[start] = 0             // Cost from start
  Vector2f current, next;
  SearchState curr_state;
  float curr_cost, score;
  while (!frontier.Empty()){
    curr_state = frontier.Pop();
    if (visited)
    current = curr_state.curr_loc;
    
    if (visited.find(current) != visited.end()) { continue; } // TODO: check if current is visited
    curr_cost = curr_state.cost;

    if (AtGoal(current)) {break;}
    vector<Vector2f> neighbors;
    Neighbors_(current, &neighbors);
    for (Vector2f next : neighbors) {
      score = GetScore_(next, current, curr_cost);
      if ()
      break;

    }


  //   for next in neighbors(current):
  //     new_cost = cost[current] + EdgeCost[current, next]
  //     if next not in cost or new_cost < cost[next]:
  //       cost[next] = new_cost  // Insertion or edit
  //       frontier.put(next, new_cost + heuristic(next))
  //       parent[next] = current
  }


}

} // namespace planner
