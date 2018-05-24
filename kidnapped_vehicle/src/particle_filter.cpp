/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <numeric>
#include <random>
#include <sstream>
#include <string>

#include "helper_functions.h"
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO(zuqqhi2): Set the number of particles.
  //   Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h
  //  for more information about this method (and others in this file).

  // Create normal distributions
  default_random_engine       gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  num_particles = 50;
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id     = i;
    p.weight = 1.0;

    // Get sample value from normal distributions
    p.x     = dist_x(gen);
    p.y     = dist_y(gen);
    p.theta = dist_theta(gen);

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity,
                                double yaw_rate) {
  // TODO(zuqqhi2): Add measurements to each particle
  //   and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution
  //  and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // Create normal distributions
  default_random_engine       gen;
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  // Move particles by motion model
  for (int i = 0; i < num_particles; i++) {
    double theta = particles[i].theta;

    if (fabs(yaw_rate) < 1e-6) {
      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);
    } else {
      particles[i].x +=
          velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta)) + dist_x(gen);
      particles[i].y +=
          velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t)) + dist_y(gen);
      particles[i].theta += yaw_rate * delta_t + dist_theta(gen);
    }
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs>  predicted,
                                     std::vector<LandmarkObs>& observations) {
  // TODO(zuqqhi2): Find the predicted measurement that
  //   is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code.
  //   But you will probably find it useful to implement
  //   this method and use it as a helper during the updateWeights phase.

  // Mapping between ground truth landmarks and observed landmarks by NN
  for (unsigned int i = 0; i < observations.size(); i++) {
    double min_dist            = 1e+10;
    int    closest_landmark_id = -1;

    // Find closest observed landmark
    for (unsigned int j = 0; j < predicted.size(); j++) {
      double cur_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      if (cur_dist < min_dist) {
        min_dist            = cur_dist;
        closest_landmark_id = predicted[j].id;
      }
    }
    observations[i].id = closest_landmark_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs>& observations,
                                   const Map&                map_landmarks) {
  // TODO(zuqqhi2): Update the weights of each particle using a mult-variate Gaussian distribution.
  //   You can read more about this distribution here:
  //   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no
  //   scaling). The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html

  for (int i = 0; i < num_particles; i++) {
    std::vector<LandmarkObs> predicted;

    // Pick up landmarks in sensor range
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      int    landmark_id = map_landmarks.landmark_list[j].id_i;
      double landmark_x  = map_landmarks.landmark_list[j].x_f;
      double landmark_y  = map_landmarks.landmark_list[j].y_f;

      if (fabs(landmark_x - particles[i].x) < sensor_range &&
          fabs(landmark_y - particles[i].y) < sensor_range) {
        predicted.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
      }
    }

    // Transform from vehicle coordinates to map
    std::vector<LandmarkObs> map_observations;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double x = cos(particles[i].theta) * observations[j].x -
                 sin(particles[i].theta) * observations[j].y + particles[i].x;
      double y = sin(particles[i].theta) * observations[j].x +
                 cos(particles[i].theta) * observations[j].y + particles[i].y;
      map_observations.push_back(LandmarkObs{observations[j].id, x, y});
    }

    // Mapping between ground truth landmark and observation
    dataAssociation(predicted, map_observations);

    particles[i].weight = 1.0;  // Initialize particle weight
    for (unsigned int j = 0; j < map_observations.size(); j++) {
      double x, y;

      // Find associated landmark position
      for (unsigned int k = 0; k < predicted.size(); k++) {
        if (predicted[k].id == map_observations[j].id) {
          x = predicted[k].x;
          y = predicted[k].y;
        }
      }

      // Update particle weight
      double coef = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
      double e1   = pow(x - map_observations[j].x, 2) / (2.0 * pow(std_landmark[0], 2));
      double e2   = pow(y - map_observations[j].y, 2) / (2.0 * pow(std_landmark[1], 2));
      particles[i].weight *= coef * exp(-(e1 + e2));
    }
  }
}

void ParticleFilter::resample() {
  // TODO(zuqqhi2): Resample particles with replacement
  //   with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // Get current particle weights to generate distribution
  std::vector<double> cur_weights;
  for (int i = 0; i < num_particles; i++) {
    cur_weights.push_back(particles[i].weight);
  }

  // Resampling by particle weights
  std::vector<Particle>        new_particles;
  default_random_engine        gen;
  std::discrete_distribution<> dd(cur_weights.begin(), cur_weights.end());
  for (int i = 0; i < num_particles; i++) {
    new_particles.push_back(particles[dd(gen)]);
  }
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                         const std::vector<double>& sense_x,
                                         const std::vector<double>& sense_y) {
  // particle: the particle to assign each listed association, and association's (x,y) world
  // coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations = associations;
  particle.sense_x      = sense_x;
  particle.sense_y      = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int>  v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s        = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream   ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s        = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream   ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s        = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
