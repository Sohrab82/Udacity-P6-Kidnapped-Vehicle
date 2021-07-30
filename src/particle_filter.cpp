#include "particle_filter.h"
#include <ctime>
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

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    // Initialize all particles to first position (based on estimates of x, y, theta and
    // their uncertainties from GPS) and all weights to 1.
    num_particles_ = 100;

    normal_distribution<double> dist_x(0, std[0]);
    normal_distribution<double> dist_y(0, std[1]);
    normal_distribution<double> dist_theta(0, std[2]);
    for (int i = 0; i < num_particles_; i++)
    {
        Particle p;
        p.set(x + dist_x(gen), y + dist_y(gen), theta + dist_theta(gen));
        p.set_noise(std[0], std[1], std[2]);
        particles_.push_back(p);
    }
    is_initialized_ = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // prediction Predicts the state for the next time step using the process model.
    for (int i = 0; i < num_particles_; i++)
    {
        particles_[i].move(delta_t, velocity, yaw_rate, dist_x(gen), dist_y(gen), dist_theta(gen));
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
    // Update the weights of each particle using a mult-variate Gaussian
    // double sum_weight = 0;
    for (int i = 0; i < num_particles_; i++)
    {
        particles_[i].update_observations(observations);
        // Finds which observations correspond to which landmarks
        // by using a nearest-neighbors data association
        particles_[i].associate_observations(map_landmarks, sensor_range);
        particles_[i].calc_weight(map_landmarks, std_landmark[0], std_landmark[1]);
        // sum_weight += particles_[i].weight_;
    }
    // for (int i = 0; i < num_particles_; i++)
    //     particles_[i].weight_ /= sum_weight;
}

void ParticleFilter::resample()
{
    /*
    Resample particles with replacement with probability proportional to their weight. 

    Slower method: random selection with weights/probabilities, and replacement(you can have every element more than once)
    sort your numbers based on their probablities
    caluclate cdf (cummulative density function)
    choose a random number from a uniform distribution (z)
    find the first number in the cdf for which cdf > z
    */
    // create a list of all the particle weights
    vector<double> weights;
    for (int i = 0; i < num_particles_; i++)
    {
        weights.push_back(particles_[i].weight_);
    }
    // find max weight
    double max_weight = -1;
    for (size_t i = 0; i < weights.size(); i++)
        if (weights[i] > max_weight)
            max_weight = weights[i];

    std::uniform_int_distribution<int> dist_index(0, num_particles_ - 1);
    std::uniform_real_distribution<double> dist_beta(0, max_weight);

    vector<Particle> resampled_particles;

    int index = dist_index(gen);
    double beta = 0;
    for (int i = 0; i < num_particles_; i++)
    {
        beta += dist_beta(gen) * 2.0;
        while (beta > weights[index])
        {
            beta -= weights[index];
            index = (index + 1) % num_particles_;
        }
        resampled_particles.push_back(particles_[index]);
    }
    particles_ = resampled_particles;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations_id_;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
    vector<double> v;

    if (coord == "X")
    {
        for (auto o : best.observations_)
            v.push_back(o.x);
    }
    else
    {
        for (auto o : best.observations_)
            v.push_back(o.y);
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}