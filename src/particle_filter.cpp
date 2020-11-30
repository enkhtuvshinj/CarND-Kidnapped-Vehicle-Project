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
#include <limits>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::default_random_engine;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/**
	 * TODO: Set the number of particles. Initialize all particles to 
	 *   first position (based on estimates of x, y, theta and their uncertainties
	 *   from GPS) and all weights to 1. 
	 * TODO: Add random Gaussian noise to each particle.
	 * NOTE: Consult particle_filter.h for more information about this method 
	 *   (and others in this file).
	 */
	if (is_initialized == false)	{
		default_random_engine gen;
		Particle p;
		
		// Set the number of particles
		num_particles = 100;  				

		// Creates a normal (Gaussian) distribution for x, y
		normal_distribution<double> dist_x(x, std[0]);
		normal_distribution<double> dist_y(y, std[1]);
		normal_distribution<double> dist_theta(theta, std[2]);
		
		// Initialize particles using Gaussian distribution
		for (int i = 0; i < num_particles; ++i) {
			p.id = i;
			p.x = dist_x(gen);
			p.y = dist_y(gen);
			p.theta = dist_theta(gen);
			p.weight = 1;
			
			particles.push_back(p);
		}
		is_initialized = true;
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
	default_random_engine gen;

	// Standard deviations are given for only x, y and yaw
	// but our task is to add Gaussian noise to velocity and yaw rate
	// Easy approach is to calculate positions (x,y) and yaw using given velocity and yaw rate.
	// then add a normal distributed noise to calculated positions and theta. 
	// For this purpose, create normal distribution with zero mean.
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_yaw(0,  std_pos[2]);
	
	for(int i=0; i<num_particles; i++)	{
		// Calculate new state
		if (fabs(yaw_rate) < 0.00001) {
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		} 
		else {
			particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}
		
		// Add the noise to newly-calculated positions
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_yaw(gen);
	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predictions, 
                                     vector<TransformedObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
	// Observations are data from sensors
	// Predictions are landmarks from map

	double minimum;
	double distance;
	int landmark_id;
	int vector_id;
	
	// Find the nearest observation for every landmark
	// If no landmark is given, then iteration doesn't work
	// If landmarks predicted, iterate throught observations for each landmark
	// Note: if landmark predicted, the nearest observation always exists in according to below algorithm.
	for (unsigned int i = 0; i < observations.size(); i++) {
		// Initialize landmark every predictions
		minimum = dist( observations[i].obs.x, observations[i].obs.y, 
						predictions[0].x,  predictions[0].y);
		landmark_id = predictions[0].id;			
		vector_id = 0;
		
		for (unsigned int j = 1; j < predictions.size(); j++) {
			// Calculate distance between observation and prediction
			distance = dist(observations[i].obs.x, observations[i].obs.y, 
							predictions[j].x,  predictions[j].y);
		
			// find the predicted landmark nearest the current observed landmark
			if (distance < minimum) {
				minimum = distance;
				landmark_id = predictions[j].id;
				vector_id = j;
			}
		}

		// Assign the observation to the nearest landmark
		observations[i].obs.id = landmark_id;
		observations[i].vec_id = vector_id;
	}
}


/**
 * Calculate Multivariate-Gaussian Probability.
 * @param 	std_landmark	- standard deviation of landmark measurement x=[0], y=[1]
 *			x_obs	- observation coordinate x
 *			y_obs	- observation coordinate y
 *			mu_x	- mean x
 *			mu_y	- mean y
 * @output 	weight	- Multivariate-Gaussian Probability
 */
double multiv_prob(	double std_landmark[], 
					double obs_x,
					double obs_y,					
					double mu_x,
					double mu_y,
					double norm) {

	// calculate exponent
	double exponent = (pow(obs_x - mu_x, 2) / (2 * pow(std_landmark[0], 2)))
					+ (pow(obs_y - mu_y, 2) / (2 * pow(std_landmark[1], 2)));
    
	// calculate weight using normalization terms and exponent
	double weight = norm * exp(-exponent);
    
	return weight;
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
	// Below steps is repeated for every particle
	// 1. Transform
	// 2. Filter landmarks within sensor range of particle
	// 3. Associate (the nearest neighbor algorithm)
	// 4. Update weight

	double temp_x;
	double temp_y;
	
	// Calculate normalization term and it is constant
	double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

	for (int i = 0; i < num_particles; i++) {
		vector<TransformedObs> transformed_observations;
		vector<LandmarkObs> predictions;

		// 1. Transform from local to global coordinates
		for (unsigned int j = 0; j < observations.size(); j++) {
			temp_x = cos(particles[i].theta)*observations[j].x - sin(particles[i].theta)*observations[j].y + particles[i].x;
			temp_y = sin(particles[i].theta)*observations[j].x + cos(particles[i].theta)*observations[j].y + particles[i].y;
			transformed_observations.push_back(TransformedObs{ observations[j].id, temp_x, temp_y, 0 });
		}

		// 2. Filter landmarks within sensor range of particle
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			temp_x = map_landmarks.landmark_list[j].x_f;
			temp_y = map_landmarks.landmark_list[j].y_f;
      
			// filter landmarks within sensor range of the particle 
			if (fabs(temp_x - particles[i].x) <= sensor_range && fabs(temp_y - particles[i].y) <= sensor_range) {
				predictions.push_back(LandmarkObs{ map_landmarks.landmark_list[j].id_i, temp_x, temp_y });
			}
		}

		// 3. Associate (the nearest neighbor algorithm)
		dataAssociation(predictions, transformed_observations);

		// 4. Update weights
		// Reset weight of particle
		particles[i].weight = 1.0;
		for (unsigned int j = 0; j < transformed_observations.size(); j++) {
			// 4.1. calculate weight for this observation with multivariate Gaussian (Determine measurement probabilities)
			double obs_weight = multiv_prob(std_landmark, 
											predictions[transformed_observations[j].vec_id].x, predictions[transformed_observations[j].vec_id].y,
											transformed_observations[j].obs.x, transformed_observations[j].obs.y,
											gauss_norm );

			// 4.2. product of this observation weight with total observations weight (Combine probabilities)
			particles[i].weight *= obs_weight;
		}
	}
}

/**
 * Find maximum weight for all particle.
 * @param 	None
 * @output 	max	- Maximum weight found
 */
double ParticleFilter::maxWeight()	{
	double max = particles[0].weight;
	for (int i = 1; i < num_particles; i++) {
		if(max < particles[i].weight)	{
			max = particles[i].weight;
		}
	}

	return max;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
	vector<Particle> resampled_particles;
	default_random_engine gen;
	
	// Random particle index
	uniform_int_distribution<int> particle_index(0, num_particles - 1);
	int index = particle_index(gen);

	double beta = 0.0;
	double max_weight = 2.0 * maxWeight();
	
	for (unsigned int i = 0; i < particles.size(); i++) {
		// Random weight
		uniform_real_distribution<double> random_weight(0.0, max_weight);
		beta += random_weight(gen);

		while (beta > particles[index].weight) {
			beta -= particles[index].weight;
			index = (index + 1) % num_particles;
		}
		resampled_particles.push_back(particles[index]);
	}
	
	// Assign the sampled particle to particles
	particles = resampled_particles;
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