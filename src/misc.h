#ifndef MISC_H
#define MISC_H

double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y);

void particle_observ_to_map(double x_p, double y_p, double theta,
                            double x_c, double y_c,
                            double &x_m, double &y_m);
double calc_dist(double x1, double y1, double x2, double y2);
#endif // MISC_H
