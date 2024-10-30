#include <stdint.h>

double comp_filter(int16_t gyro_y_rate, double acc_x_angle, double dt);

double mahony_filter(int16_t gyro_y_rate, double acc_x_angle, double dt);

double kalman_filter(int16_t gyro_y_rate, double acc_x_angle, double dt);
