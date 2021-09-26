//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "visualization/visualization.h"
#include "amrl_msgs/VisualizationMsg.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Rotation2Df;
using vector_map::VectorMap;
using std::sin;
using std::cos;
using amrl_msgs::VisualizationMsg;

DEFINE_uint32(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(-1000, -1000),
    prev_odom_angle_(-1000),
    odom_initialized_(false) {
    }

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

/*
// a_loc_in_b and a_angle_in_b is the position of frame A in frame B.
// object_loc_in_a and object_angle_in_a is the position of the object in frame A.
// Calculates the position of the object in frame B.
void ParticleFilter::TransformAToB(const Vector2f& a_loc_in_b, float a_angle_in_b,
                               const Vector2f& object_loc_in_a, float object_angle_in_a,
                               Vector2f* loc_ptr, float* angle_ptr) {
  float& new_angle = *angle_ptr;
  Vector2f& new_loc = *loc_ptr;
  
  Rotation2Df r(a_angle_in_b);
  new_loc = object_loc_in_a + r * a_loc_in_b;
  new_angle = a_angle_in_b + object_angle_in_a;
}
*/

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr,
                                            vector<line2f>* laser_lines) {
  vector<Vector2f>& scan = *scan_ptr;
  vector<line2f>& lasers = *laser_lines;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  lasers.resize(num_ranges);
  // Fill in the entries of scan
  float step_size = (angle_max - angle_min) / num_ranges;
  Rotation2Df r(-angle);
  Vector2f mLaserLoc = loc + r * kLaserLoc;
  for (size_t i = 0; i < scan.size(); i++) {
    float angle_i = angle_min + i * step_size + angle;
    Rotation2Df r_i(-angle_i);
    Vector2f v_min = r_i * Vector2f(range_min, range_min) + mLaserLoc;
    Vector2f v_max = r_i * Vector2f(range_max, range_max) + mLaserLoc;
    line2f laser_line(v_min.x(), v_min.y(), v_max.x(), v_max.y());

    Vector2f intersection_point (HORIZON * cos(angle_i), HORIZON * sin(angle_i));
    Vector2f intersection_point_tmp (0.0, 0.0);
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];

      // Check for intersections
      bool intersects = map_line.Intersection(laser_line, &intersection_point_tmp);

      if (!intersects) continue;
      
      bool closer = false;
      if (abs(intersection_point_tmp.x() - mLaserLoc.x()) < abs(intersection_point.x() - mLaserLoc.x()) ) {
        closer = true;
      } else if (abs(intersection_point_tmp.y() - mLaserLoc.y()) < abs(intersection_point.y() - mLaserLoc.y()) ) {
        closer = true;
      }

      if(closer) {
        intersection_point.x() = intersection_point_tmp.x();
        intersection_point.y() = intersection_point_tmp.y();
      }
    }
    scan[i] = intersection_point;
  }
}

// returns the log likelihood of x in a Gaussian distribution
float calculateLogGaussian(float mean, float stddev, float x) {
  // TODO: update to more robust model
  // remember we don't want to use 0 outside the window
  return - 0.5 * pow(x - mean, 2) / pow(stddev, 2);
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  vector<Vector2f> predicted_cloud;
  vector<line2f> lasers;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(), range_min, range_max, 
                         angle_min, angle_max, &predicted_cloud, &lasers);
  float w = 0.0;
  for (size_t i = 0; i < ranges.size(); ++i) {
    // Get the actual range
    float r = ranges[i];
    if (r < range_min || r > range_max) {printf("something went wrong on ranges\n");} // for debug
    
    // Get the predicted range
    float a = angle_min + i * (angle_max - angle_min) / ranges.size();
    Rotation2Df rotation(a);
    Vector2f mLaserLoc = p_ptr->loc + rotation * kLaserLoc;
    float x = abs(predicted_cloud[i].x() - mLaserLoc.x());
    float y = abs(predicted_cloud[i].y() - mLaserLoc.y());
    float predicted_r = sqrt(pow(x, 2) + pow(y, 2));

    w += calculateLogGaussian(r, SENSOR_STD_DEV, predicted_r);
  }

  p_ptr->weight = pow(M_E, w * GAMMA);
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights
  
  // TODO resample: 1 )low-variance resampling 2) resample/update less often

  vector<Particle> new_particles;
  for (size_t i = 0; i < particles_.size(); ++i) {
    float rand = rng_.UniformRandom(0, 1);
    size_t j = 0;
    while (rand > 0.0) {
      rand -= particles_[j].weight;
      j++;
    }
    if(rand == 0.0) {
      j++;
    }
    struct Particle p = { Vector2f(particles_[j-1].loc.x(), particles_[j-1].loc.y()), 
                          particles_[j-1].angle, 1.0 / particles_.size()};
    new_particles.push_back(p);
  }
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // TODO: This should only do anything when the robot has moved 0.15m or rotated 10 degrees

  float sum = 0.0;
  for (struct Particle p : particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &p);
    sum += p.weight;
  }
  // Normalizes the weights
  for (struct Particle p : particles_) {
    p.weight /= sum;
  }

  Resample();
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  std::vector<Particle> new_particles_;
  // if (debug) printf("odom_loc.x: %.2f, odom_loc.y: %.2f, odom_angle: %.2f\n", 
  //     odom_loc.x(), odom_loc.y(), odom_angle);
  for (struct Particle p : particles_) {
    // TODO: prev_odom or prev_particle_odom
    Rotation2Df r_map_odom(p.angle-prev_odom_angle_);
    Vector2f new_loc = prev_odom_loc_.x() == -1000 ? p.loc 
          : p.loc + r_map_odom * (odom_loc - prev_odom_loc_);
    float new_angle  = prev_odom_angle_ == -1000? p.angle : p.angle + odom_angle - prev_odom_angle_;
    
    float new_x = rng_.Gaussian(new_loc.x(), MOTION_X_STD_DEV);
    float new_y = rng_.Gaussian(new_loc.y(), MOTION_Y_STD_DEV);
    float new_a = rng_.Gaussian(new_angle, MOTION_A_STD_DEV);

    struct Particle new_p = {Vector2f(new_x, new_y), new_a, p.weight};
    new_particles_.push_back(new_p);
  }
  particles_ = new_particles_;

  // Update prev_odom
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  // TODO: sample a few particles from a gaussian around loc and angle
  // const float DELTA_X = 0.05;
  // const float DELTA_Y = 0.05;
  // const float DELTA_A = 0.05;
  // Assume all particles are at the exact location provided
  particles_.clear();
  for (size_t i = 0; i < FLAGS_num_particles; i++) {
    // float x = rng_.Gaussian(loc.x(), DELTA_X);
    // float y = rng_.Gaussian(loc.y(), DELTA_Y);
    // float a = rng_.Gaussian(angle, DELTA_A);

    struct Particle p = {Vector2f(loc.x(), loc.y()), angle, 1.0 / FLAGS_num_particles};
    particles_.push_back(p);
  }

  // TODO: check this
  // Update prev_odom
  prev_odom_angle_ = -1000;
  prev_odom_loc_ = Vector2f(-1000, -1000);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
// TODO: Fix me.
  float x_sum = 0;
  float y_sum = 0;
  float a_sum = 0;
  for (struct Particle p : particles_) {
    x_sum += p.loc.x();
    y_sum += p.loc.y();
    a_sum += p.angle;
  }
  loc = Vector2f(x_sum / particles_.size(), y_sum / particles_.size());
  angle = a_sum / particles_.size();
}

}  // namespace particle_filter

/*
1. Does it reset odometry when initialize particle filter?
*/
