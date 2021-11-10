#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "vector_map/vector_map.h"

using namespace std;
using namespace Eigen;

#ifndef PLANNER_H_
#define PLANNER_H_

namespace planner {

const float GRID_SIZE = 0.5;

struct SearchState {
  Vector2f curr_loc;
  float cost;
  SearchState* parent;
} SearchState;

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

private:
  

	Vector2f global_goal_mloc_;
	float global_goal_mangle_;
  bool global_goal_set_;
  vector<Vector2f> lattices_;

  vector_map::VectorMap map_;
  vector<Vector2f> path;
  // constexpr static float GRID_SIZE = 0.5;

  // get the heuristic score of a location
  float GetScore_(const Vector2f& loc, const Vector2f& prev_loc, float prev_cost);
  void Neighbors_(const Vector2f& loc, vector<Vector2f>* neighbors);
  bool AtGoal(const Vector2f& robot_mloc);
};

} // namespace planner

#endif // PLANNER_H_