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

#include <map>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 10;  //10

	default_random_engine gen;
	double std_x, std_y, std_psi; // Standard deviations for x, y, and psi




	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std[0]);

	// TODO: Create normal distributions for y and psi
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_psi(theta, std[2]);

	//particles.resize(num_particles);
	for (int i = 0; i < num_particles; i++) {

		Particle newParticle;
		newParticle.id = i ;
		newParticle.x = dist_x(gen) ;
		newParticle.y = dist_y(gen) ;
		newParticle.theta = dist_psi(gen);
		newParticle.weight = 1.0f ;

		// Print your samples to the terminal.
		//cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_psi << endl;

		particles.push_back(newParticle) ;
	}
	std::cout << "init no. of particles = " << particles.size() << std::endl;
	weights.resize(num_particles);

	is_initialized = true ;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::cout << "--------------prediction----------------------" << std::endl ;



	// Set up Gaussian noise, as in `ParticleFilter::init`
	default_random_engine gen;

	double new_x, new_y, new_theta ;

	for (int i = 0; i < num_particles; i++) {
		if ( yaw_rate == 0) {
			new_x = particles[i].x + (velocity * delta_t * cos(particles[i].theta)) ;
			new_y = particles[i].y + (velocity * delta_t * sin(particles[i].theta)) ;
			new_theta = particles[i].theta ;
		} else {
			new_x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)) ;
			new_y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)) ;
			new_theta = particles[i].theta + yaw_rate * delta_t ;
		}

		normal_distribution<double> n_x(new_x, std_pos[0]);
		normal_distribution<double> n_y(new_y, std_pos[1]);
		normal_distribution<double> n_theta(new_theta, std_pos[2]);

		particles[i].x = n_x(gen) ;
		particles[i].y = n_y(gen) ;
		particles[i].theta = n_theta(gen) ;

		std::cout << "prediction : for particles[" << i << "] predict x= : " << particles[i].x << " y= " << particles[i].y << " theta= " << particles[i].theta << std::endl ;
	}





}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	std::cout << "-------------dataAssociation---------------" << std::endl ;

	//vector<LandmarkObs> associations;

	// for each observed measurement
	for (int i = 0 ; i < observations.size(); i++) {

		// make code easier to read
		LandmarkObs &obs = observations[i];
		std::cout << "---------observations[" << i << "]-------- \n has obs.x = " << obs.x << "obs.y = " << obs.y << std::endl;

		// find predicted measurement closest to observation
		// initialise minimum distance prediction (pred closest to obs)
		//LandmarkObs min = predicted[0];
		double  min_distance_squared = 99999999.0;
		//std::cout << "min.x = " << min.x << "min.y = " << min.y << std::endl;

		//double min_distance_squared = pow(min.x - obs.x, 2) + pow(min.y - obs.y, 2);
		//std::cout <<"min_distance_squared = " << min_distance_squared << std::endl;

		// for each prediction
		for (int j = 0; j < predicted.size(); j++) {
			std::cout << "predicted[" << j << "] has id= " << predicted[j].id << " x= " << predicted[j].x << " y = " << predicted[j].y << std::endl;

			// calculate distance between predicted measurement and obs
			double distance_squared = pow(predicted[j].x - obs.x, 2) + pow(predicted[j].y - obs.y, 2);
			std::cout << "distance_squared = " << distance_squared << std::endl;

			if (distance_squared < min_distance_squared) {
				//min = predicted[j];
				//std::cout << "min change to be min.x= " << min.x << " min.y= " << min.y << std::endl;
				min_distance_squared = distance_squared ;

				obs.id = predicted[j].id;
				std::cout << "min belongs to prediction id= " << predicted[j].id << std::endl;
			}
		}

		//associations.push_back(min);
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations, Map map_landmarks) {
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

	std::cout << "-------------updateWeights---------------" << std::endl ;
#if 0
	double var_x = pow(std_landmark[0], 2);
	double var_y = pow(std_landmark[1], 2);
	double covar_xy = std_landmark[0] * std_landmark[1];
	double weights_sum = 0;

	for (int i = 0; i < num_particles; i++) {

		std::vector<LandmarkObs> transfered_map_landmarks;

		Particle tPart = particles[i]; // get particle
		double px = tPart.x;
		double py = tPart.y;
		double ptheta = tPart.theta;

		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			Map::single_landmark_s temp_landmark_input = map_landmarks.landmark_list[j];
			LandmarkObs temp_landmark_output;
			temp_landmark_output.id = temp_landmark_input.id_i;
			double sin_ptheta = sin(ptheta - M_PI / 2); // angle change according to README
			double cos_ptheta = cos(ptheta - M_PI / 2); // angle change according to README
			double delta_x = temp_landmark_input.x_f - px;
			double delta_y = temp_landmark_input.y_f - py;
			temp_landmark_output.x = -delta_x * sin_ptheta + delta_y * cos_ptheta;
			temp_landmark_output.y = -delta_x * cos_ptheta - delta_y * sin_ptheta;
			transfered_map_landmarks.push_back(temp_landmark_output);
		}

		std::sort(transfered_map_landmarks.begin(), transfered_map_landmarks.end());


	}
#endif

	vector<LandmarkObs> transformed_obs;
	transformed_obs.resize(observations.size());

	for (int i = 0; i < num_particles; i++) {

		Particle &particle = particles[i]; // reference of a particle
		std::cout << "updateWeights : for particles[" << i << "] use x= : " << particle.x << " y= " << particle.y << " theta= " << particle.theta << " to clac in range landmarks !" << std::endl ;

		for (int j = 0; j < observations.size(); j++) {
			std::cout << "updateWeights : for observations[" << j << "] has x= " << observations[j].x << " y= " << observations[j].y << std::endl ;

			LandmarkObs transformed_ob;
			// transform from vehicle coordinate to map coordinate
			transformed_ob.x = observations[j].x * cos(particle.theta) - observations[j].y * sin(particle.theta) + particle.x;
			transformed_ob.y = observations[j].x * sin(particle.theta) + observations[j].y * cos(particle.theta) + particle.y;

			// leave transformed_ob.id to be the most likely landmark.id_i !!

			std::cout << "updateWeights : for observations[" << j << "] transformed x= " << transformed_ob.x << " y= " << transformed_ob.y << std::endl ;

			transformed_obs[j] = transformed_ob;
		}
		std::cout << "updateWeights : for particles[" << i << "] has size of transformed_obs : " << transformed_obs.size() << std::endl ;

		std::vector<LandmarkObs> inrange_landmarks;

		std::map<int, Map::single_landmark_s> idx2landmark;

		for (const auto& landmark : map_landmarks.landmark_list) {
			double distance = dist(landmark.x_f, landmark.y_f, particle.x, particle.y);
			std::cout << "updateWeights : for particles[" << i << "] to landmark i= " << landmark.id_i << " has distance= " << distance << std::endl ;

			if (distance <= sensor_range) {

				inrange_landmarks.push_back(LandmarkObs{ landmark.id_i, landmark.x_f, landmark.y_f });

				idx2landmark.insert(std::make_pair(landmark.id_i, landmark)); // log the landmark so that it can be used later
				//std::cout << "updateWeights : insert landmark i= " << landmark.id_i << " x= " << landmark.x_f << " y= " << landmark.y_f << std::endl ;
			}
		}
		std::cout << "updateWeights : for particles[" << i << "] has size of inserted landmarks : " << idx2landmark.size() << std::endl ;

		if (inrange_landmarks.size() > 0) {

			// for this particle[i], match each of its transformed observations to each in_range landmarks
			dataAssociation(inrange_landmarks, transformed_obs);

			particle.weight = 1.0; //reset the weight of the particle

			std::cout << "updateWeights : for particles[" << i << "] has reset weight : " << particle.weight << " | " << particles[i].weight << std::endl ;

			for (const auto observation : transformed_obs) {
				std::cout << "updateWeights after dataAssociation: for observation x= " << observation.x << " y= " << observation.y << " the most likely landmark is id= : " << observation.id << " at x= " << idx2landmark[observation.id].x_f << " y=" << idx2landmark[observation.id].y_f << std::endl ;

				double mu_x = idx2landmark[observation.id].x_f;
				double mu_y = idx2landmark[observation.id].y_f;
				double x = observation.x;
				double y = observation.y;
				double s_x = std_landmark[0];
				double s_y = std_landmark[1];
				double x_diff = (x - mu_x) * (x - mu_x) / (2 * s_x * s_x);
				double y_diff = (y - mu_y) * (y - mu_y) / (2 * s_y * s_y);
				particle.weight *= 1 / (2 * M_PI * s_x * s_y) * exp(-(x_diff + y_diff));
			}
			weights[i] = particle.weight;
		} else {
			weights[i] = 0.0;
		}

		std::cout << "updateWeights : for particles[" << i << "] has weight : " << particle.weight << " | " << particles[i].weight << " | " << weights[i] << std::endl ;

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::random_device rd;
	std::mt19937 gen(rd());
	std::discrete_distribution<> d(weights.begin(), weights.end());
	vector<Particle> particles_new;

	particles_new.resize(num_particles);
	for (int n = 0; n < num_particles; ++n) {
		particles_new[n] = particles[d(gen)];
		std::cout << "resample : new particle at n= " << n << " with id= " << particles_new[n].id << std::endl ;
	}

	particles = particles_new;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}




Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y) {
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
