#include "particle.h"

void Particle::init(double gps_x, double gps_y, double theta,
                    double std_x, double std_y, double std_theta)
{
    std::default_random_engine gen;

    // This line creates a normal (Gaussian) distribution for x
    normal_distribution<double> dist_x(0, std_x);
    normal_distribution<double> dist_y(0, std_y);
    normal_distribution<double> dist_theta(0, std_theta);

    // Sample from these normal distributions like this:
    x_ = gps_x + dist_x(gen);
    y_ = gps_y + dist_y(gen);
    theta_ = theta + dist_theta(gen);
    // where "gen" is the random engine initialized earlier.

    std::cout << "Initialized: " << x_ << " " << y_ << " "
              << theta_ << std::endl;

    return;
}

void Particle::move(double d_t, double vel, double theta_dot, double noise_x, double noise_y, double noise_theta)
{

    if ((std::fabs(theta_dot) < 1.0e-4) || (std::isinf(theta_dot)))
    {
        x_ = x_ + vel * d_t * cos(theta_) + noise_x;
        y_ = y_ + vel * d_t * sin(theta_) + noise_y;
        theta_ = theta_ + noise_theta;
    }
    else
    {
        x_ = x_ + vel / theta_dot * (sin(theta_ + theta_dot * d_t) - sin(theta_)) + noise_x;
        y_ = y_ + vel / theta_dot * (cos(theta_) - cos(theta_ + theta_dot * d_t)) + noise_y;
        theta_ = theta_ + theta_dot * d_t + noise_theta;
    }
    // std::cout << "Moved: " << x_ << " " << y_ << " "
    //           << theta_ << std::endl;
}

double Particle::rmse_position(double g_x, double g_y)
{
    return sqrt(pow((x_ - g_x), 2.) + pow((y_ - g_y), 2.));
}

double Particle::rmse_theta(double g_theta)
{
    return sqrt(pow(theta_ - g_theta, 2.));
}

void Particle::set(double x, double y, double theta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
    std::cout << "Set to: " << x_ << " " << y_ << " "
              << theta_ << std::endl;
}

void Particle::set_noise(double std_x, double std_y, double std_theta)
{
    std_x_ = std_x;
    std_y_ = std_y;
    std_theta_ = std_theta;
}

void Particle::update_observations(const std::vector<LandmarkObs> &measurements)
{
    // constructs observations_ vector which contains the "measurements" in
    // map coordinate system
    observations_.clear();
    associations_id_.clear();
    double x_m, y_m;
    for (size_t i = 0; i < measurements.size(); i++)
    {
        particle_observ_to_map(x_, y_, theta_,
                               measurements[i].x, measurements[i].y,
                               x_m, y_m);
        observations_.push_back({measurements[i].id, x_m, y_m});
        associations_id_.push_back(-1);
    }
}

void Particle::associate_observations(const Map &map, double sensor_range)
{
    double dist;
    double closest_dist;
    int closest_id;
    for (size_t i = 0; i < observations_.size(); i++)
    {
        closest_dist = 1e6;
        closest_id = -1;
        for (size_t j = 0; j < map.landmark_list.size(); j++)
        {
            // distance from particle to the landmark
            dist = calc_dist(x_, y_,
                             map.landmark_list[j].x_f, map.landmark_list[j].y_f);
            if (dist > sensor_range)
            {
                // landmarks not in range of the particle
                continue;
            }

            dist = calc_dist(observations_[i].x, observations_[i].y,
                             map.landmark_list[j].x_f, map.landmark_list[j].y_f);
            if (dist < closest_dist)
            {
                closest_dist = dist;
                closest_id = map.landmark_list[j].id_i;
            }
        }
        // std::cout << "closeset id:" << closest_id << std::endl;
        if (closest_id == -1)
        {
            std::cout << "No landmark found " << std::endl;
        }
        associations_id_[i] = closest_id;
    }
}

void Particle::calc_weight(const Map &map, double std_landmark_x, double std_landmark_y)
{
    if (associations_id_.size() == 0)
    {
        // no sensor measurements
        std::cout << "SOMETHING IS WRONG!" << std::endl;
        weight_ = 0;
        return;
    }
    double p;
    weight_ = 1;
    int match;
    for (size_t i = 0; i < associations_id_.size(); i++)
    {
        if (associations_id_[i] == -1)
        {
            std::cout << "Not assigned" << std::endl;
            continue;
        }
        for (size_t j = 0; j < map.landmark_list.size(); j++)
            if (map.landmark_list[j].id_i == associations_id_[i])
            {
                match = j;
                break;
            }
        p = multiv_prob(std_landmark_x, std_landmark_y,
                        observations_[i].x, observations_[i].y,
                        map.landmark_list[match].x_f, map.landmark_list[match].y_f);
        // std::cout << observations_[i].x << " " << observations_[i].y << " "
        //           << map.landmark_list[match].x_f << " " << map.landmark_list[match].y_f << " " << p << std::endl;
        weight_ *= p;
    }
    // std::cout << weight_ << std::endl;
}
