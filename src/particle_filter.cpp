/**
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 * Created on: Dec 12, 2016
 * Author : Tiffany Huang
 * Modified: member function added by Subir Das / Jan 12 2020
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
using std::normal_distribution; //for Gaussian distribution

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  
  // Resize vector of particles
  particles.resize(num_particles);

  std::default_random_engine gen;

  // Create normal (Gaussian) distributions for x,y and theta(yaw angle)
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for(int i=0;i<num_particles;++i){
    Particle p_temp; 
    p_temp.id=i;
    p_temp.x=dist_x(gen);
    p_temp.y=dist_y(gen);
    p_temp.theta=dist_theta(gen);
    p_temp.weight=1;
    particles.push_back(p_temp);
  }
  is_initialized=true;
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
  std::default_random_engine gen; 

  // Create normal (Gaussian) distributions for x,y and theta(yaw angle)
  normal_distribution<double> dist_x(0, std_pos[0]); //centered at zero
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for (int i=0;i<num_particles;++i){

    if (fabs(yaw_rate) <0.00001 ){ //vehicle direction is not changing
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } else {
      // adding mesurements to each particles
      double A=particles[i].theta + (yaw_rate * delta_t);
      particles[i].x += (velocity/yaw_rate)* (sin(A) -sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) -cos(A));
      particles[i].theta +=yaw_rate *delta_t;
    }
  
    // adding random Gaussian noise
    particles[i].x +=dist_x(gen);
    particles[i].y +=dist_y(gen);
    particles[i].theta +=dist_theta(gen);
  }
}


// void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
//                                      vector<LandmarkObs>& observations) {
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<Observation>& transformedObs) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  double min_dist=std::numeric_limits<double>::max(); //sensor_range;
  for(unsigned i=0;i<transformedObs.size();++i){
    
    int landmark_id=-1;
    double landmark_x=0.01;
    double landmark_y=0.01;

    double x_obs=transformedObs[i].x;
    double y_obs=transformedObs[i].y;


    for(unsigned j=0;j<predicted.size();++j){
      double distance=dist(x_obs,y_obs,predicted[j].x,predicted[j].y);
      if(distance<min_dist){
        min_dist=distance;
        landmark_id=predicted[j].id;
        landmark_x=predicted[j].x;
        landmark_y=predicted[j].y;
      }
    }
    transformedObs[i].landmark_id=landmark_id;
    transformedObs[i].landmark_x=landmark_x;
    transformedObs[i].landmark_y=landmark_y;
  }
}


// void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
//                                    const vector<LandmarkObs> &observations, 
//                                    const Map &map_landmarks) {
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<Observation> &observations, 
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
  for (int i=0;i<num_particles;++i){
    double x_part=particles[i].x;
    double y_part=particles[i].y;
    double theta_part=particles[i].theta;

    // observation measurement transformations
    for (unsigned j=0;j<observations.size();++j){
      double x_map=x_part + (cos(theta_part) * observations[j].x) - (sin(theta_part) * observations[j].y);
      double y_map=y_part + (sin(theta_part) * observations[j].x) + (cos(theta_part) * observations[j].y);
      particles[i].transformedObs.push_back(Observation{observations[j].id, x_map, y_map});
    }
    // identifying measurement and landmark associations
    for (unsigned k=0;k<map_landmarks.landmark_list.size();++k){
      double distance=dist(x_part,y_part,map_landmarks.landmark_list[k].x_f,map_landmarks.landmark_list[k].y_f);
      if(distance<=(sensor_range*sensor_range)){
      //if(fabs(particles[i].x-map_landmarks.landmark_list[k].x_f)<=sensor_range && fabs(particles[i].y-map_landmarks.landmark_list[k].y_f)<=sensor_range){
        particles[i].predictionObs.push_back(LandmarkObs{map_landmarks.landmark_list[k].id_i,map_landmarks.landmark_list[k].x_f,map_landmarks.landmark_list[k].y_f});
      }
    }
    // calling data association to find the matching landmak with sensor observation
    dataAssociation(particles[i].predictionObs,particles[i].transformedObs);

    // calculate each particle's weight for this observation with multivariate Gaussian
    // calculate each particle's weight
    // ultimate goal is to find a weight parameter for each particle that represents 
    // how well that particle fits to being in the same location as the actual car
    // Reseting weight.
    particles[i].weight = 1.0; //resetting weight
    for (unsigned l=0;l<particles[i].transformedObs.size();++l){

      double x_obs=particles[i].transformedObs[l].x;
      double y_obs=particles[i].transformedObs[l].y;
      double mu_x=particles[i].transformedObs[l].landmark_x;
      double mu_y=particles[i].transformedObs[l].landmark_y;

      // calculate normalization term
      double gauss_norm;
      double stdev_x=std_landmark[0];
      double stddev_y=std_landmark[1];
      gauss_norm = 1 / (2 * M_PI * stdev_x * stddev_y);

      // calculate exponent
      double exponent;
      exponent = (pow(x_obs- mu_x, 2) / (2 * pow(stdev_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(stddev_y, 2)));

      // product of this weight with total observations weight
      double weight=gauss_norm * exp(-exponent);
      particles[i].weight *= weight;
    }
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector <Particle> resampled_particles;
  vector <double> weights;

  //collect all particle weights into a vector of double
  for(unsigned i=0;i<particles.size();++i){
    weights.push_back(particles[i].weight);
  }
  // generating uniform distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  
  std::uniform_int_distribution<> disIndex(0, num_particles-1);
  
  int index=disIndex(gen);
  double beta=0.0;
  double max_weight = *max_element(weights.begin(), weights.end());
  std::uniform_real_distribution<double> disBeta(0.0, 1.0);

  // resampling using the wheel approach
  for (unsigned j=0;j<particles.size();++j){
    beta +=disBeta(gen) *2.0 * max_weight;
    while(beta>weights[index]){
      beta -=weights[index];
      index =(index +1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }
  particles=resampled_particles;
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