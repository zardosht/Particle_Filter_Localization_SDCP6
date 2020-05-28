/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::sin;
using std::cos;

std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  if(is_initialized){
    return;
  }

  num_particles = 1000;  // TODO: Set the number of particles
  
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for(unsigned int i = 0; i < num_particles; ++i){
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    double weight = 1.0;
    particle.weight = weight;
    particles.push_back(particle);
    weights.push_back(weight);
  }

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

    double v_over_yawrate = velocity / yaw_rate;
    double yawrate_dt = yaw_rate * delta_t;

    for (Particle particle : particles) {
      // previous values of x, y, theta for particle
      double x0 = particle.x;
      double y0 = particle.y;
      double theta0 = particle.theta;

      // calcuate perfect values for x, y, theta by 
      // applying the motion equations and assuming no noise
      double pure_theta = theta0 + yawrate_dt;
      double pure_x = x0 + v_over_yawrate * (sin(pure_theta) - sin(theta0));
      double pure_y = y0 + v_over_yawrate * (cos(theta0) - cos(pure_theta));

      // define normal distributions for adding noise arround 
      // perfect values of x,y,theta
      normal_distribution<double> dist_x(pure_x, std_pos[0]);
      normal_distribution<double> dist_y(pure_y, std_pos[1]);
      normal_distribution<double> dist_theta(pure_theta, std_pos[2]);

      // set the new x,y,theat values for particle from noisy distributions
      particle.x = dist_x(gen);
      particle.y = dist_y(gen);
      particle.theta = dist_theta(gen);
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for (LandmarkObs obs : observations) {
    if (obs.id == -1) {
      // find the nearest landmark to this observation
      // and associate the id of the landmark with the observation
      double min_dist = std::numeric_limits<double>::max();
      for (LandmarkObs landmark : predicted) {
        double current_dist = dist(landmark.x, landmark.y, obs.x, obs.y);
        if (current_dist < min_dist) {
          min_dist = current_dist;
          obs.id = landmark.id;
        }
      }
    }
  }


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

   
   // for each particle
   for (unsigned int i = 0; i < particles.size(); ++i) {
      Particle particle = particles[i];
      double x_p = particle.x;
      double y_p = particle.y;
      double theta_p = particle.theta;

      // find the landmarks near the predicted 
      // postion of the particle
      vector<Map::single_landmark_s> nearby_landmarks;
      for (Map::single_landmark_s landmark : map_landmarks.landmark_list) {
         if (dist(landmark.x_f, landmark.y_f, x_p, y_p) < sensor_range) {
           nearby_landmarks.push_back(landmark);
         }
      }

      // bring the observations to the map coordinates 
      // for the particle
      vector<LandmarkObs> transformed_observations; 
      for (LandmarkObs obs : observations) {
         LandmarkObs transformed_observation;
         transformed_observation.id = -1;
         transformed_observation.x = (cos(theta_p) * obs.x) + (-sin(theta_p) * obs.y) + x_p;
         transformed_observation.y = (sin(theta_p) * obs.x) + (cos(theta_p) * obs.y) + y_p;
         transformed_observations.push_back(transformed_observation);
      }

      // associated transformed observations with 
      // the landmarks around particle to get a list
      // of (landmark, observation) pairs. 
      // Note 1: The function does not get a list of landmarks as input,
      //         but a list of LandmarkObs objects. So, we have to consturct
      //         this list from nearby landmarks. 
      // Note 2: The function does not return list of pairs, 
      //         but instead associates the id of the landmark 
      //         with the observation. 
      vector<LandmarkObs> nearby_landmark_obss;
      for (Map::single_landmark_s landmark : nearby_landmarks) {
        nearby_landmark_obss.push_back(
          LandmarkObs{
            landmark.id_i, 
            landmark.x_f,
            landmark.y_f
          }
        );
      }
      dataAssociation(nearby_landmark_obss, transformed_observations);
      
      // set associations for particle for debugging
      vector<int> associations;
      vector<double> sense_x;
      vector<double> sense_y;
      for (LandmarkObs obs : transformed_observations) {
        associations.push_back(obs.id);
        sense_x.push_back(obs.x);
        sense_y.push_back(obs.y);
      }
      SetAssociations(particle, associations, sense_x, sense_y);


      // Calcuate the wieght of particle by multiplying the 
      // probabilities for each (landmark, observation) pair.
      // Assuming Gaussian noise for the observations, this probability 
      // tells us how likely is it that the particles sees this observation for this landmark. 
      // The observation is transformed to the map coordinate _of the particls_.
      // The landmark postion is also in map coordinate. 
      // If the the observation is made at a correct position (i.e. the predicted position 
      // of the particle is near the real position of the car that made the measurement), 
      // then this observation is a value almost equal to the real position of its 
      // associated landmark (the mean of the Gaussian). 
      double weight_p = 1.0;
      for (LandmarkObs trans_obs : transformed_observations) {
         for (LandmarkObs landmark_obs : nearby_landmark_obss) {
           if (trans_obs.id == landmark_obs.id) {
             weight_p *= multiv_gauss(trans_obs.x, trans_obs.y, 
                                      landmark_obs.x, landmark_obs.y, 
                                      std_landmark[0], std_landmark[1]);
           }
         }
      }
      particle.weight = weight_p;
      weights[i] = weight_p;
   }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  std::discrete_distribution<int> discrete_dist(weights.begin(), weights.end());
  vector<Particle> new_particles(num_particles);
  vector<double> new_weights(num_particles);
  for (unsigned int i = 0; i < num_particles; ++i) {
    unsigned int selected_index = discrete_dist(gen);
    Particle selected_particle = particles[selected_index];
    new_particles.push_back(selected_particle);
    new_weights.push_back(selected_particle.weight);
  }
  particles = new_particles;
  weights = new_weights;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}