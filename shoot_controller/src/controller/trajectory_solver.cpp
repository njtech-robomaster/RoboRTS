#include <cmath>
#include <vector>

double calc_polynomial(const std::vector<double> &coeffs, double x) {
	double y = 0;
	for (size_t k = 0; k < coeffs.size(); k++) {
		y += coeffs[k] * std::pow(x, k);
	}
	return y;
}

double calc_polynomial_derivative(const std::vector<double> &coeffs, double x) {
	double y = 0;
	for (size_t k = 1; k < coeffs.size(); k++) {
		y += k * coeffs[k] * std::pow(x, k - 1);
	}
	return y;
}

double newton_iteration(const std::vector<double> &coeffs, double x0,
                        double *err_ptr = nullptr, double max_err = 1e-6,
                        int max_iterations = 100) {
	int iterations = 0;
	double y;
	while (y = calc_polynomial(coeffs, x0), std::abs(y) > max_err) {
		if (iterations++ > max_iterations) {
			break;
		}
		x0 = x0 - y / calc_polynomial_derivative(coeffs, x0);
	}
	if (err_ptr != nullptr) {
		*err_ptr = y;
	}
	return x0;
}

const double g = 9.8;

double compute_pitch(double distance, double height, double gun_barrel_length,
                     double bullet_velocity) {
	std::vector<double> coeffs = {
	    -2 * g * gun_barrel_length * distance +
	        2 * height * bullet_velocity * bullet_velocity +
	        g * distance * distance + g * gun_barrel_length * gun_barrel_length,
	    -4 * bullet_velocity * bullet_velocity * distance,
	    -2 * g * gun_barrel_length * distance + 3 * g * distance * distance -
	        2 * height * bullet_velocity * bullet_velocity -
	        g * gun_barrel_length * gun_barrel_length,
	    +8 * bullet_velocity * bullet_velocity * distance,
	    +2 * g * gun_barrel_length * distance + 3 * g * distance * distance -
	        2 * height * bullet_velocity * bullet_velocity -
	        g * gun_barrel_length * gun_barrel_length,
	    -4 * bullet_velocity * bullet_velocity * distance,
	    +2 * g * gun_barrel_length * distance +
	        2 * height * bullet_velocity * bullet_velocity +
	        g * distance * distance +
	        g * gun_barrel_length * gun_barrel_length};
	double err;
	double k = newton_iteration(coeffs, 0, &err);
	if (err > 1e-3) {
		return NAN;
	}
	return 2 * std::atan(k);
}
