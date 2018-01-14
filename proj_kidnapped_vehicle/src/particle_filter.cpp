/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;
	default_random_engine gen;
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for (int i = 0; i < num_particles; i ++)
	{
		double sample_x = dist_x(gen);
		double sample_y = dist_y(gen);
		double sample_theta = dist_theta(gen);
		particles.push_back(Particle(i, sample_x, sample_y, sample_theta));
		weights.push_back(1.0);
	}

	is_initialized = true;
}

void ParticleFilter::prediction
	(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	for (int i = 0; i < num_particles; i ++)
	{
		double x0 = particles[i].x;
		double y0 = particles[i].y;
		double theta0 = particles[i].theta;


		double x1, y1, theta1;
		if (abs(yaw_rate) > 1.0e-5)
		{
			x1 = x0 + velocity / yaw_rate *(sin(theta0 + yaw_rate * delta_t) - sin(theta0));
			y1 = y0 + velocity / yaw_rate *(cos(theta0) - cos(theta0 + yaw_rate * delta_t));
		}
		else
		{
			x1 = x0 + velocity * sin(theta0) * delta_t;
			y1 = y0 + velocity * cos(theta0) * delta_t;
		}
		theta1 = theta0 + yaw_rate * delta_t;

		
		normal_distribution<double> dist_x(x1, std_x);
		normal_distribution<double> dist_y(y1, std_y);
		normal_distribution<double> dist_theta(theta1, std_theta);

		x1 = dist_x(gen);
		y1 = dist_y(gen);
		theta1 = dist_theta(gen);

		particles[i].x = x1;
		particles[i].y = y1;
		particles[i].theta = theta1;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// this is for one particle,
	// predicted: the predicted measurements for each landmark, measured from this particle filter position
	//            size is landmark_size
	// observations: the truelly measured result, whose size will be less than landmark_size
	// the purpose of this function is to update observation id to the matched predicted order
	for (int i = 0; i < observations.size(); i ++)
	{
		double x1 = observations[i].x;
		double y1 = observations[i].y;
		double min_distance = -1.0;
		int matched_landmark_id = -1;
		for (int j = 0; j < predicted.size(); j ++)
		{
			double x2 = predicted[j].x;
			double y2 = predicted[j].y;
			double distance = dist(x1, y1, x2, y2);
			if (min_distance < 0.0 || min_distance > distance)
			{
				min_distance = distance;
				matched_landmark_id = j;
			}
		}
		observations[i].id = matched_landmark_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	double weights_sum = 0.0;

	for (int i = 0; i < num_particles; i ++)
	{
		double particle_x = particles[i].x;
		double particle_y = particles[i].y;
		double particle_theta = particles[i].theta;

		// calc predicted observations for each land mark, whih will be in the map coordinate
		std::vector<LandmarkObs> predicted;
		for (int j = 0; j < map_landmarks.landmark_list.size(); j ++)
		{
			int    landmark_id = map_landmarks.landmark_list[j].id_i;
			double landmark_x  = map_landmarks.landmark_list[j].x_f;
			double landmark_y  = map_landmarks.landmark_list[j].y_f;
			
			LandmarkObs obs;
			obs.id = landmark_id;
			obs.x = landmark_x;// - particle_x;
			obs.y = landmark_y;// - particle_y;
			predicted.push_back(obs);
		}

		// transfer the observations from car coordinate to map coordinate
		std::vector<LandmarkObs> map_coord_observations; 
		for (int j = 0; j < observations.size(); j ++)
		{
			// int landmark_id = observations[j].id;
			double car_coord_obs_x = observations[j].x;
			double car_coord_obs_y = observations[j].y;

			double map_coord_obs_x = 
				cos(particle_theta) * car_coord_obs_x - sin(particle_theta) * car_coord_obs_y + particle_x;
			double map_coord_obs_y = 
				sin(particle_theta) * car_coord_obs_x + cos(particle_theta) * car_coord_obs_y + particle_y;
			
			LandmarkObs map_obs;
			map_obs.x = map_coord_obs_x;
			map_obs.y = map_coord_obs_y;
			map_obs.id = 0;
			map_coord_observations.push_back(map_obs);
		}

		// now we have the predicted obs for each landmarks, and truely obs detected,
		// both obs are in the same map coordinate

		dataAssociation(predicted, map_coord_observations);
		// now the map_coord_observations id has been matched to the landmarks

		// update particle associations, sense_x, sense_y
		vector<int>   associations;
		vector<double> sense_x;
		vector<double> sense_y;

		// get the weight for this particle, by multiply each obs probability
		double weight = 1.0;
		for (int j = 0; j < map_coord_observations.size(); j ++)
		{
			double map_coord_obs_x = map_coord_observations[j].x;
			double map_coord_obs_y = map_coord_observations[j].y;
			int matched_landmark_id = map_coord_observations[j].id;

			// get the matched landmark prediction
			LandmarkObs matched_obs = predicted[matched_landmark_id];

			double map_coord_pred_x = matched_obs.x;
			double map_coord_pred_y = matched_obs.y;

			associations.push_back(matched_obs.id);
			sense_x.push_back(matched_obs.x);// + particles[i].x);
			sense_y.push_back(matched_obs.y);// + particles[i].y); 

			double gaussian_norm = 0.5 / (std_landmark[0] * std_landmark[1]);
			double exponent = - pow((map_coord_obs_x - map_coord_pred_x),2) / (2 * pow(std_landmark[0],2)) 
				- pow((map_coord_obs_y - map_coord_obs_y),2) / (2 * pow(std_landmark[1],2));
			weight *= gaussian_norm * exp(exponent);
		}
		weights[i] = weight;
		weights_sum += weight;

		particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);
		particles[i].weight = weight;

	}

	// normalize the weights
	for (int i = 0; i < weights.size(); i ++)
		weights[i] /= weights_sum;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::vector<Particle> new_particles;
	default_random_engine gen;
	discrete_distribution<> dist(weights.begin(), weights.end());
	for (int i = 0; i < num_particles; i ++)
	{
		int sample = dist(gen);
		new_particles.push_back(particles[sample]);
	}
	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(
	Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, 
	//          and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
