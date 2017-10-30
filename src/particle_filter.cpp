/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *      Submission: Tan Wang
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
#include <assert.h>

#include "particle_filter.h"

using namespace std;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
   	num_particles = 50;
    	default_random_engine gen;
    
    	normal_distribution<double> dist_x(x, std[0]);
    	normal_distribution<double> dist_y(y, std[1]);
    	normal_distribution<double> dist_theta(theta, std[2]);
    
    	for(int i=0; i<num_particles; i++){
        	double sample_x = dist_x(gen);
        	double sample_y = dist_y(gen);
        	double sample_theta = dist_theta(gen);
        	Particle p = {i, sample_x, sample_y, sample_theta, 1};
        	weights.push_back(1);
        	particles.push_back(p);
    	}

    	is_initialized = true;
    	cout << "INIT DONE ....\n";
    
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    
    
    	default_random_engine gen;
    
    	for(int i=0; i<particles.size(); i++){
        	double x = particles[i].x;
        	double y = particles[i].y;
        	double theta = particles[i].theta;
        
        	double x_next, y_next, theta_next;
        
        	// calculate the car at the next time step
        	if(fabs(yaw_rate) > 0.001){
            	x_next = x + (velocity / yaw_rate) * (sin(theta+yaw_rate*delta_t) - sin(theta));
            	y_next = y + (velocity / yaw_rate) * (-cos(theta+yaw_rate*delta_t) + cos(theta));
            	theta_next = theta + delta_t*yaw_rate;
        	}else{
            	x_next = x + velocity*delta_t*cos(theta);
            	y_next = y + velocity*delta_t*sin(theta);
            	theta_next = theta;
        	}
        
        
        	// add noise
        	normal_distribution<double> dist_x(x_next, std_pos[0]);
        	normal_distribution<double> dist_y(y_next, std_pos[1]);
        	normal_distribution<double> dist_theta(theta_next, std_pos[2]);
        
       	double next_x = dist_x(gen);
        	double next_y = dist_y(gen);
        	double next_theta = dist_theta(gen);
        
        	// update the particle
        	particles[i].x = next_x;
        	particles[i].y = next_y;
        	particles[i].theta = next_theta;
    	}
	
	cout << "PREDICTION DONE ....\n";
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    
    	for(int i=0; i<observations.size(); i++){
        	int minLandmarkIndex = 0;
        
        	// if your compiler does not work change INFINITY to a very large number
        	double minDistance = INFINITY;
        	for(int j=0; j<predicted.size(); j++){
            	double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
            	if(distance < minDistance){
                	minDistance = distance;
                	minLandmarkIndex = j;
            	}
        	}
        	observations[i].id = minLandmarkIndex;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {    
    
    	double sig_x = std_landmark[0];
    	double sig_y = std_landmark[1];
    
    
    	for(int i=0; i<particles.size(); i++){
        	double x = particles[i].x;
        	double y = particles[i].y;
        	double theta = particles[i].theta;
        	double weight = 1;

        	// transform the coordinates
        	vector<LandmarkObs> transformed_observations = vector<LandmarkObs>(observations.size());
        	for(int i = 0; i< observations.size(); i++){
            	double x_map= x + (cos(theta) * observations[i].x) - (sin(theta) * observations[i].y);
            	double y_map= y + (cos(theta) * observations[i].y) + (sin(theta) * observations[i].x);
            
            	LandmarkObs trans = {i, x_map, y_map};
            	transformed_observations[i] = trans;
          }
        
        	// reorganize landmark data 
        	vector<LandmarkObs> ground_truth = vector<LandmarkObs>(map_landmarks.landmark_list.size());
        	for(int i=0; i< map_landmarks.landmark_list.size(); i++){
            	auto ldm = map_landmarks.landmark_list[i];
            	ground_truth[i] = {ldm.id_i, ldm.x_f, ldm.y_f};
        	}
        
        	// associate data ( transformed_observation will be changed)
        	dataAssociation(ground_truth, transformed_observations);
        
        
        	// calculate weight
        	for(int i=0; i<transformed_observations.size(); i++){
        	    	double gauss_norm= (1.0/(2 * M_PI * sig_x * sig_y));
            	double x_obs = transformed_observations[i].x;
            	double y_obs = transformed_observations[i].y;
            	double mu_x = ground_truth[transformed_observations[i].id].x;
            	double mu_y = ground_truth[transformed_observations[i].id].y;
            	double exponent= pow((x_obs - mu_x),2)/(2 * sig_x*sig_x) + pow(y_obs - mu_y,2)/(2 * sig_y*sig_y);
            
            	double weightUpdate = gauss_norm * exp(-exponent);
            	weight *= weightUpdate;
            
    		}
        
      	particles[i].weight = weight;
        	weights[i] = weight;
 
    	}
	
	cout << "UPDATE WEIGHTS DONE ....\n";

	

}

void ParticleFilter::resample() {
    
    	vector<Particle> newParticles = vector<Particle>(num_particles);
    	vector<double> newWeights = vector<double>(num_particles);
    	default_random_engine gen;
    	discrete_distribution<int> dist_index(weights.begin(), weights.end());
    
    	for(int i=0; i<num_particles; i++){
        	int indexToBeAdded = dist_index(gen);
        	Particle np = particles[indexToBeAdded];
        	np.id = i;
        	newParticles[i] = np;
        	newWeights[i] = particles[indexToBeAdded].weight;
    	}
    
    	// ready for next iteration
    	particles = newParticles;
	weights = newWeights; 
	cout << "RESAMPLE DONE ....\n";
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
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
