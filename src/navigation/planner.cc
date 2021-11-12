#include <cmath>
#include <unordered_set>
#include <unordered_map>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Pose2Df.h"
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
using geometry::line2f;

namespace
{
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
  return path_;
}

void Planner::SetGlobalGoal(const Vector2f &loc, float angle) {
  cout << "SetGlobalGoal" << endl;
  global_goal_mloc_ = loc;
  global_goal_mangle_ = angle;
  global_goal_set_ = true;
}

float Planner::GetCost_(const Vector2f& loc, const Vector2f& prev_loc, float prev_cost) {  
  return prev_cost + (prev_loc - loc).norm();
}

float Planner::GetHeuristic_(const Vector2f& loc) {  
  return (global_goal_mloc_ - loc).norm();
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

// implements the A* algorithm to find the best path to the goal
void Planner::GetGlobalPlan(const Vector2f& robot_mloc, float robot_mangle) {
  
  if (!global_goal_set_) { return; }

  // priority queue stores <SearchState, score>
  SimpleQueue<struct SearchState, float> frontier;
  struct SearchState start_state;
  start_state.curr_loc = robot_mloc;
  start_state.cost = 0.0;
  frontier.Push(start_state, 0.0);
  
  // keep track of visited search states
  unordered_set<Vector2f, Vector2fHash> visited;
  // keep track of the parents of each search states
  // key: child, value: parent
  unordered_map<Vector2f, Vector2f, Vector2fHash> parents;

  Vector2f current, next;
  struct SearchState curr_state;
  float curr_cost, cost, heuristic;
  while (!frontier.Empty()){
    curr_state = frontier.Pop();
    current = curr_state.curr_loc;
    visited.insert(current);
    // if (visited.find(current) != visited.end()) { continue; } 
    curr_cost = curr_state.cost;

    if (AtGoal(current)) { break; }
    vector<Vector2f> neighbors;
    Neighbors_(current, &neighbors);
    for (Vector2f& next : neighbors) {
      if (visited.find(next) != visited.end()) { continue; }
      cost = GetCost_(next, current, curr_cost);
      heuristic = GetHeuristic_(next);
      struct SearchState next_state;
      next_state.curr_loc = next;
      next_state.cost = cost;
      if (frontier.Push(next_state, cost + heuristic)) {
        parents.insert_or_assign(next, current);
      }
    }
  }

  // construct path
  path_.clear();
  if (!AtGoal(current)) { return; }
  
  path_.insert(path_.begin(), current);
  while (current != start_state.curr_loc) {
    current = parents.at(current);
    path_.insert(path_.begin(), current);
  }
}

void Planner::VisualizePath(VisualizationMsg& global_viz_msg) {
  if (!global_goal_set_) { return; }

  // cout << "path size: " << path_.size() << endl;
  for (size_t i = 0; i < path_.size() - 1; ++i) {
    // printf("(%f, %f) --> ", path_[i].x(), path_[i].y());
    visualization::DrawLine(path_[i], path_[i+1], 0x000000, global_viz_msg);
  }
  // cout << endl;

  visualization::DrawCross(global_goal_mloc_, 0.3, 0xFF0000, global_viz_msg);
}

} // namespace planner
