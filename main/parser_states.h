#include <stdint.h>

static void start(uint8_t param);

static void motor_velocity(uint8_t param);

static void motor_velocity_param_1(uint8_t param);

static void motor_velocity_param_2(uint8_t param);

static void set_pid(uint8_t param);

static void set_pid_kp(uint8_t param);

static void set_pid_ki(uint8_t param);

static void set_pid_kd(uint8_t param);

static void set_angle_correction(uint8_t param);

static void set_angle_correction_theta(uint8_t param);

static void set_angle_correction_speed(uint8_t param);

static int wheels_direction = 0; 
static int is_theta_counter_clock_wise = 0;
static void (*next_state)(int) = start; 