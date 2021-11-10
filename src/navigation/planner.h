#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using namespace std;
using namespace Eigen;

#ifndef PLANNER_H_
#define PLANNER_H_

namespace planner {

class Planner {

public:
  Planner();
  void SetGlobalGoal(const Vector2f &loc, float angle);
  void GetGlobalPlan(const Vector2f& odom_loc, float odom_angle);
	const vector<Vector2f>& GetPath();

private:
  // get the heuristic score of a location
  float getScore(const Vector2f& loc, const Vector2f& prev_loc, float prev_cost);

	Vector2f global_goal_mloc_;
	float global_goal_mangle_;
  bool global_goal_set_;

  vector<Vector2f> path;

};

} // namespace planner

#endif // PLANNER_H_