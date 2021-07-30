#ifndef PARTICLE_H
#define PARTICLE_H

#include <iostream>
#include <random> // Need this for sampling from distributions
#include "misc.h"
#include "map.h"
#include "helper_functions.h"

using std::normal_distribution;

class Particle
{
public:
    double x_, y_, theta_;
    double std_x_, std_y_, std_theta_;
    std::vector<LandmarkObs> observations_;
    std::vector<int> associations_id_;
    double weight_;

    Particle()
    {
        weight_ = 1;
    };
    ~Particle(){};
    void init(double gps_x, double gps_y, double theta,
              double std_x, double std_y, double std_theta);
    void set_noise(double std_x, double std_y, double std_theta);
    void move(double d_t, double vel, double theta_dot, double noise_x, double noise_y, double noise_theta);
    double rmse_position(double g_x, double g_y);
    double rmse_theta(double g_theta);
    void set(double x, double y, double theta);
    void update_observations(const std::vector<LandmarkObs> &measurements);
    void associate_observations(const Map &map, double sensor_range);
    void calc_weight(const Map &map, double std_landmark_x, double std_landmark_y);

private:
};

#endif