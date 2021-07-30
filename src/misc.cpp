#include <cmath>
#include "misc.h"

double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y)
{
    // calculate normalization term
    double gauss_norm;
    gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

    // calculate exponent
    double exponent;
    exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2))) + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

    // calculate weight using normalization terms and exponent
    double weight;
    weight = gauss_norm * exp(-exponent);

    return weight;
}

void particle_observ_to_map(double x_p, double y_p, double theta,
                            double x_c, double y_c,
                            double &x_m, double &y_m)
{
    // transforms point (x_c, y_c) in particle coordiante system
    // to map coordinate system (x_m, y_m)
    // particle is at (x_p, y_p) in the map coord sys
    x_m = x_p + x_c * cos(theta) - y_c * sin(theta);
    y_m = y_p + x_c * sin(theta) + y_c * cos(theta);
}

double calc_dist(double x1, double y1, double x2, double y2)
{
    double d_x, d_y;
    d_x = x1 - x2;
    d_y = y1 - y2;
    return sqrt(d_x * d_x + d_y * d_y);
}