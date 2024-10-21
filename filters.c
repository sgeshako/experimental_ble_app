#include "filters.h"

double comp_filter(int16_t gyro_y_rate, double acc_x_angle, double dt)
{
	static double gyro_y_angle = 0;

	double gyro_error = 0.01 * (gyro_y_angle - acc_x_angle);
	gyro_y_angle += gyro_y_rate * 0.00875f * dt;
	gyro_y_angle -= gyro_error;
	return gyro_y_angle;
//	return -100 * gyro_error;
}

double comp_filter_2(int16_t gyro_y_rate, double acc_x_angle, double dt)
{
	static double gyro_y_angle = 0;
	static double Kp = 0.01;

	gyro_y_angle += gyro_y_rate * 0.00875f * dt;
	double gyro_error = gyro_y_angle - acc_x_angle;
	gyro_y_angle -= Kp * gyro_error;
	return gyro_y_angle;
}

double mahony_filter(int16_t gyro_y_rate, double acc_x_angle, double dt)
{
	static double gyro_y_angle = 0;
	static double error_integration = 0;
//	static double Kp = 0.9;
//	static double Ki = 0.8;
	static double Kp = 4.9;
	static double Ki = 1.1;

	double gyro_error = acc_x_angle - gyro_y_angle;
	error_integration += Ki * dt * gyro_error;
	gyro_y_angle += (Kp * gyro_error + error_integration + gyro_y_rate * 0.00875f) * dt;
	return gyro_y_angle;
//	return gyro_error;
}

static double Q_acc_angle_variance = 0.001;
static double Q_gyro_bias_variance = 0.003;
static double R_meas_noise_variance = 0.03;

double kalman_filter(int16_t gyro_y_rate, double acc_x_angle, double dt)
{
	static double gyro_y_angle = 0;
	static double gyro_bias = 0;
	static double P[2][2]; // Error covariance matrix of the a priori estimate
	static double S = 0; // Innovation covariance
	static double K[2]; // Kalman gain matrix

	// Calculate a priori estimates
	gyro_y_angle += dt * (gyro_y_rate * 0.00875f - gyro_bias);

	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_acc_angle_variance);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += dt * Q_gyro_bias_variance;

	// Calculate a posteriori estimates
	double y_innovation = acc_x_angle - gyro_y_angle;
	S = P[0][0] + R_meas_noise_variance;
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	gyro_y_angle += K[0] * y_innovation;
	gyro_bias += K[1] * y_innovation;

	P[1][0] -= K[1] * P[0][0];
	P[1][1] -= K[1] * P[0][1];

	P[0][0] -= K[0] * P[0][0];
	P[0][1] -= K[0] * P[0][1];

	return gyro_y_angle;
//	return gyro_bias;
}
