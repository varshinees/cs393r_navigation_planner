#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "vector_map/vector_map.h"
#include "amrl_msgs/VisualizationMsg.h"

using namespace std;
using namespace Eigen;
using amrl_msgs::VisualizationMsg;

#ifndef PLANNER_H_
#define PLANNER_H_

namespace planner {

const float GRID_SIZE = 0.1;
const float CIRCLE_RADIUS = 2.0;
const float STOP_DIST = 0.5;

struct SearchState {
  Vector2f curr_loc;
  float cost;

  bool operator==(const SearchState &other) const
  { return curr_loc.x() == other.curr_loc.x() && curr_loc.y() == other.curr_loc.y(); }
};

class Planner {

struct Vector2fHash {
  size_t operator() (Vector2f const& v) const noexcept {
    size_t h1 = hash<float>{}(v.x());
    size_t h2 = hash<float>{}(v.y());
    return h1 ^ (h2 << 1);
  }

};

public:
  Planner();
  void SetMap(const string &map_file);
  void SetGlobalGoal(const Vector2f &loc, float angle);
  void GetGlobalPlan(const Vector2f& odom_loc, float odom_angle);
	const vector<Vector2f>& GetPath();
  Vector2f GetLocalGoal(const Vector2f& robot_mloc, float robot_mangle);
  bool AtGoal(const Vector2f& robot_mloc);
  void VisualizePath(VisualizationMsg& global_viz_msg);

private:
	Vector2f global_goal_mloc_;
	float global_goal_mangle_;
  bool global_goal_set_;
  vector<Vector2f> lattices_;

  vector_map::VectorMap map_;
  vector<Vector2f> path_;
  size_t path_start_idx;
  // constexpr static float GRID_SIZE = 0.5;

  // get the heuristic score of a location
  float GetCost_(const Vector2f& loc, const Vector2f& prev_loc, float prev_cost);
  float GetHeuristic_(const Vector2f& loc);
  void Neighbors_(const Vector2f& loc, vector<Vector2f>* neighbors);
};

} // namespace planner

#endif // PLANNER_H_