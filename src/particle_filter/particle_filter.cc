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

CONFIG_FLOAT(GAMMA, "GAMMA");
CONFIG_FLOAT(SENSOR_STD_DEV, "SENSOR_STD_DEV");
CONFIG_FLOAT(D_SHORT, "D_SHORT");
CONFIG_FLOAT(D_LONG, "D_LONG");
CONFIG_FLOAT(P_OUTSIDE_RANGE, "P_OUTSIDE_RANGE");
CONFIG_FLOAT(MOTION_X_STD_DEV, "MOTION_X_STD_DEV");
CONFIG_FLOAT(MOTION_Y_STD_DEV, "MOTION_Y_STD_DEV");
CONFIG_FLOAT(MOTION_A_STD_DEV, "MOTION_A_STD_DEV");
CONFIG_FLOAT(MOTION_DIST_K1, "MOTION_DIST_K1");
CONFIG_FLOAT(MOTION_DIST_K2, "MOTION_DIST_K2");
CONFIG_FLOAT(MOTION_A_K1, "MOTION_A_K1");
CONFIG_FLOAT(MOTION_A_K2, "MOTION_A_K2");
CONFIG_FLOAT(MAX_D_DIST, "MAX_D_DIST");
CONFIG_FLOAT(MAX_D_ANGLE, "MAX_D_ANGLE");

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
 * loc_ab and angle_ab is the position of frame A in frame B.
 * loc_pa and angle_pa is the position of the object p in frame A.
 * Calculates the position of p in frame B.
 */
void TransformAToB(const Vector2f& loc_ab, float angle_ab,
                   const Vector2f& loc_pa, float angle_pa,
                   Vector2f* loc_ptr, float* angle_ptr) {
  float& new_angle = *angle_ptr;
  Vector2f& new_loc = *loc_ptr;
  
  Rotation2Df r(angle_ab);
  new_loc = loc_ab + r * loc_pa;
  new_angle = angle_ab + angle_pa;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  
  // Get laser frame's position relative to the map frame.
  Rotation2Df r(angle);
  Vector2f mLaserLoc = loc + r * kLaserLoc;
  float mLaserAngle = angle;

  // Fill in the entries of scan
  float step_size = (angle_max - angle_min) / num_ranges;
  for (size_t i = 0; i < scan.size(); i++) {
    // Transform points location from laser frame to map frame
    float angle_i = angle_min + i * step_size;
    Rotation2Df r_i(angle_i);
    Vector2f v_min = r_i * Vector2f(range_min, 0);
    Vector2f v_max = r_i * Vector2f(range_max, 0);
    
    Vector2f loc_min(0,0);
    float angle_min = 0.0;
    Vector2f loc_max(0,0);
    float angle_max = 0.0;
    TransformAToB(mLaserLoc, mLaserAngle, v_min, angle_i, &loc_min, &angle_min);
    TransformAToB(mLaserLoc, mLaserAngle, v_max, angle_i, &loc_max, &angle_max);

    line2f laser_line(loc_min.x(), loc_min.y(), loc_max.x(), loc_max.y());

    // Assumes the intersection point is at horizon if no intersection detected
    Vector2f v_horizon = r_i * Vector2f(HORIZON, 0);
    Vector2f intersection_point(0,0);
    float angle_default = 0.0;
    TransformAToB(mLaserLoc, mLaserAngle, v_horizon, angle_i, &intersection_point, &angle_default);
    
    // Get closest interesction
    Vector2f intersection_point_tmp (0.0, 0.0);
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];
      bool intersects = map_line.Intersection(laser_line, &intersection_point_tmp);
      if (!intersects) continue;
      
      bool closer = false;
      if (abs(intersection_point_tmp.x() - mLaserLoc.x()) < abs(intersection_point.x() - mLaserLoc.x()) )
        closer = true;
      else if (abs(intersection_point_tmp.y() - mLaserLoc.y()) < abs(intersection_point.y() - mLaserLoc.y()))
        closer = true;
      
      if(closer) {
        intersection_point.x() = intersection_point_tmp.x();
        intersection_point.y() = intersection_point_tmp.y();
      }
    }
    scan[i] = intersection_point;
  }
}

// returns the log likelihood of x in a Gaussian distribution
float calculateLogGaussian(float mean, float x, float stddev, float range_min, float range_max) {
  // remember we don't want to use 0 outside the window
  if (x < range_min || x > range_max) {
    return CONFIG_P_OUTSIDE_RANGE;
  } else if (x < mean - CONFIG_D_SHORT) {
    return - pow(CONFIG_D_SHORT, 2) / pow(stddev, 2);
  } else if (x > mean + CONFIG_D_LONG) {
    return - pow(CONFIG_D_LONG, 2) / pow(stddev, 2);
  } else {
    return - pow(x - mean, 2) / pow(stddev, 2);
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Predict the expected observations for the particle
  vector<Vector2f> predicted_cloud;
  size_t downsample_rate = 20;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size() / downsample_rate, range_min, range_max, 
                         angle_min, angle_max, &predicted_cloud);
  
  // Assign weights to the particles based on the observation likelihood
  float w = 0.0;
  for (size_t i = 0; i < ranges.size() / downsample_rate; ++i) {
    // Get the actual range
    float actual_r = ranges[i * downsample_rate];
    // Ignore invalid readings
    if(actual_r < range_min || actual_r > range_max)
      continue;
    
    // Get the predicted range
    Rotation2Df rotation(p_ptr->angle);
    Vector2f mLaserLoc = p_ptr->loc + rotation * kLaserLoc;
    float x = predicted_cloud[i].x() - mLaserLoc.x();
    float y = predicted_cloud[i].y() - mLaserLoc.y();
    float predicted_r = sqrt(pow(x, 2) + pow(y, 2));

    w += calculateLogGaussian(actual_r, predicted_r, CONFIG_SENSOR_STD_DEV, range_min, range_max);
  }
  p_ptr->weight = w * CONFIG_GAMMA;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights
  float w_sum = 0.0;
  for(Particle &p : particles_) {
    w_sum += p.weight;
  }
  vector<Particle> new_particles;
  float rand = rng_.UniformRandom(0, w_sum);
  float step = w_sum / FLAGS_num_particles;
  for (size_t i = 0; i < particles_.size(); ++i) {
    rand = rand >= w_sum ? rand - w_sum : rand;
    size_t j = 0;
    float rand_cp = rand;
    while (rand_cp >= 0.0) {
      rand_cp -= particles_[j].weight;
      j++;
    }
    struct Particle p = { Vector2f(particles_[j-1].loc.x(), particles_[j-1].loc.y()), 
                          particles_[j-1].angle, 1.0 / particles_.size()};
    new_particles.push_back(p);
    rand += step;
  }
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  
  // This should only do anything when the robot has moved 0.15m or rotated 10 degrees
  if (d_dist < CONFIG_MAX_D_DIST && d_angle < CONFIG_MAX_D_ANGLE)
    return;
  
  d_dist = 0.0;
  d_angle = 0.0;

  // Updates
  double w_max = -std::numeric_limits<double>::max();
  for (Particle &p : particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &p);
    w_max = p.weight > w_max ? p.weight : w_max;
  }
  updated = !updated;

  // Normalizes the weights
  for (Particle &p : particles_)
    p.weight = pow(M_E, p.weight - w_max);

  if (!updated)
    Resample();
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  std::vector<Particle> new_particles_;
  Vector2f odom_delta = abs(prev_odom_loc_.x() + 1000) < kEpsilon ? Vector2f(0.0, 0.0) : odom_loc - prev_odom_loc_;
  float dist_delta = sqrt(pow(odom_delta.x(), 2) + pow(odom_delta.y(), 2));
  float a_delta = prev_odom_angle_ == -1000 ? 0.0 : odom_angle - prev_odom_angle_;
  a_delta = a_delta > M_PI ? a_delta - 2 * M_PI : (a_delta <= -M_PI ? a_delta + 2 * M_PI : a_delta);
  for (struct Particle p : particles_) {
    // Get a new particle
    Vector2f new_loc = p.loc;
    float new_angle = p.angle;
    if(prev_odom_angle_ != -1000) {
      Rotation2Df r(p.angle - prev_odom_angle_);
      new_loc = p.loc + r * odom_delta;
      new_angle = p.angle + a_delta;
      if (new_angle > M_PI)
        new_angle -= 2 * M_PI;
      if (new_angle <= -M_PI)
        new_angle += 2 * M_PI;
    }

    // Add noises
    // dist_delta < 1.0, -M_PI < a_delta <= M_PI
    float new_x = rng_.Gaussian(new_loc.x(), CONFIG_MOTION_DIST_K1 * dist_delta + CONFIG_MOTION_DIST_K2 * abs(a_delta) );
    float new_y = rng_.Gaussian(new_loc.y(), CONFIG_MOTION_DIST_K1 * dist_delta + CONFIG_MOTION_DIST_K2 * abs(a_delta) );
    
    float std_a = CONFIG_MOTION_A_K1 * dist_delta + CONFIG_MOTION_A_K2 * abs(a_delta);
    std_a = std_a > M_PI_2 ? M_PI_2 : std_a;
    
    float new_a = rng_.Gaussian(new_angle, std_a);
    new_a = new_a > M_PI ? new_a - 2 * M_PI : (new_a < -M_PI ? new_a + 2 * M_PI : new_a);

    struct Particle new_p = {Vector2f(new_x, new_y), new_a, p.weight};
    new_particles_.push_back(new_p);
  }
  particles_ = new_particles_;

  // update dist travelled since last update
  d_dist += dist_delta;
  d_angle += a_delta;
  
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
  // Sample particles from a gaussian around loc and angle
  particles_.clear();
  for (size_t i = 0; i < FLAGS_num_particles; i++) {
    float x = rng_.Gaussian(loc.x(), CONFIG_MOTION_X_STD_DEV);
    float y = rng_.Gaussian(loc.y(), CONFIG_MOTION_Y_STD_DEV);
    float a = rng_.Gaussian(angle, CONFIG_MOTION_A_STD_DEV);
    a = a > M_PI ? a - 2 * M_PI : (a <= -M_PI ? a + 2 * M_PI : a);

    struct Particle p = {Vector2f(x, y), a, 1.0 / FLAGS_num_particles};
    particles_.push_back(p);
  }

  // Update prev_odom
  prev_odom_angle_ = -1000;
  prev_odom_loc_ = Vector2f(-1000, -1000);
  d_dist = 0.0;
  d_angle = 0.0;
  updated = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  float x_sum = 0;
  float y_sum = 0;
  float cos_a_sum = 0.0;
  float sin_a_sum = 0.0;
  for (struct Particle p : particles_) {
    x_sum += p.loc.x();
    y_sum += p.loc.y();
    cos_a_sum += cos(p.angle);
    sin_a_sum += sin(p.angle);
  }
  loc = Vector2f(x_sum / particles_.size(), y_sum / particles_.size());
  angle = atan2(sin_a_sum / particles_.size(), cos_a_sum / particles_.size());
}

}  // namespace particle_filter
