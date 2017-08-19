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
	num_particles = 1000;  //10

	default_random_engine gen;
	double std_x, std_y, std_psi; // Standard deviations for x, y, and psi




	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std[0]);

	// TODO: Create normal distributions for y and psi
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_psi(theta, std[2]);


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

	is_initialized = true ;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Set up Gaussian noise, as in `ParticleFilter::init`
	default_random_engine gen;
	normal_distribution<double> noise_x(0.0, std_pos[0]);
	normal_distribution<double> noise_y(0.0, std_pos[1]);
	normal_distribution<double> noise_theta(0.0, std_pos[2]);

	if (fabs(yaw_rate) < 0.0001) {
		yaw_rate = 0.0001;

		// check with motion model
		for (auto && particle : particles) {

			particle.x += (velocity / yaw_rate) * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta)) + noise_x(gen);
			particle.y += (velocity / yaw_rate) * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t)) + noise_y(gen);
			particle.theta += yaw_rate * delta_t + noise_theta(gen);
		}

	} else {  //fabs(yaw_rate) < 0.0001, i.e. yaw_rate ~= 0
		for (auto && particle : particles) {

			particle.x += (velocity * delta_t * cos(particle.theta)) + noise_x(gen);
			particle.y += (velocity * delta_t * sin(particle.theta)) + noise_y(gen);
			particle.theta = particle.theta + noise_theta(gen);
		}
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.


	// for each observed measurement
	for (int i=0 ; i< observations.size(); i++) {

		// make code easier to read
		LandmarkObs obs = observations[i];
		std::cout << "observations[i] has obs.x = " << obs.x << "obs.y = " << obs.y << std::endl;

		// find predicted measurement closest to observation
		// initialise minimum distance prediction (pred closest to obs)
		LandmarkObs min = predicted[0];
		std::cout << "min.x = " << min.x << "min.y = " << min.y << std::endl;

		double min_distance_squared = pow(min.x - obs.x, 2) + pow(min.y - obs.y, 2);
		std::cout <<"min_distance_squared = " << min_distance_squared << std::endl;

		// for each prediction
		for (int j=0; j < predicted.size(); j++) {
			std::cout <<"predicted[j] has x= " << predicted[j].x <<" y = " << predicted[j].y << std::endl;

			// calculate distance between predicted measurement and obs
			double distance_squared = pow(predicted[j].x - obs.x, 2) + pow(predicted[j].y - obs.y, 2);
			std::cout <<"distance_squared = " << distance_squared << std::endl;

			if (distance_squared < min_distance_squared) {
				min = predicted[j];
				std::cout << "min change to be m.x= " << min.x << " min.y= " << min.y << std::endl;
			}
		}

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
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
