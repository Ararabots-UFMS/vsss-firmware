#include "esp_log.h"
// #include "freertos/task.h"

#include "PIDController.h"
#include "state_machine_definitions.h"
#include "parser.h"
#include "definitions.h"


static int wheels_direction = 0;
static int param1 = 0;
static int param2 = 0;
static int current_state = START;
static int first_operation = 0;
static int second_operation = 0;
static int sum_for_next_operation[4] = {START, MOTOR_VELOCITY_PARAM_1, SET_ANGLE_CORRECTION_THETA, SET_PID_KP};
static int rotation_direction = 0;
static uint32_t kp=0, ki=0, kd=0;

#ifdef DEBUG
static char msg_walk[4][19] = {
	"Andando pra frente",
	"Left: 0   Right: 1",
	"Left: 1   Right: 0",
	"Andando pra tras.."
};
static char angle[4][4] = {
	" CW",
	"   ",
	"   ",
	"CCW"
};
#endif

extern PIDCONTROLLER pid_controller;
extern TaskHandle_t motorTaskHandler;

motorPackage parser_motor_package;
extern void writeMotorPackage(motorPackage*);

void parser_params(uint8_t received_param){

    #ifdef DEBUG
        ESP_LOGW("I received:", "%d", received_param);
    #endif

	first_operation = current_state & 82; // 01010010
	// Bit mask for reducing the if comparassions

	switch (first_operation) {
		case START: // State 00000000
			second_operation = (received_param & 12)>>2; // Bit mask for reducing if comparassions in
			// sum_for_next_operation at position second_operation
			// this allows for the code to jump to next state
			wheels_direction = received_param & 3; // Wheels orientation
      rotation_direction = received_param & 2;
			current_state = sum_for_next_operation[second_operation];
			#ifdef DEBUG
				ESP_LOGE("State:", "Start\n");
			#endif
			break;

		case MOTOR_VELOCITY_PARAM_1:
			if ( current_state == MOTOR_VELOCITY_PARAM_1){
				#ifdef DEBUG
					ESP_LOGE("State:", "MOTOR_VELOCITY_PARAM_1\n");
				#endif
				param1 = received_param;
				current_state = MOTOR_VELOCITY_PARAM_2;
			}else{
				#ifdef DEBUG
					ESP_LOGE("State:", "MOTOR_VELOCITY_PARAM_2 %s\n", msg_walk[wheels_direction] );
				#endif
				param2 = received_param;
				// MOTOR FORCE YEAH
				parser_motor_package.packetID++;
				parser_motor_package.wheels_direction = wheels_direction; // Direction
				parser_motor_package.speed_l = param1; // Left speed
				parser_motor_package.speed_r = param2; // right speed
				parser_motor_package.control_type = 1; // Old or new control type
				writeMotorPackage(&parser_motor_package);
				xTaskNotifyGive(motorTaskHandler);
				current_state = START;
			}
			break;

		case SET_ANGLE_CORRECTION_THETA://> 7 && < 15:
			if ( current_state == SET_ANGLE_CORRECTION_THETA){
				#ifdef DEBUG
					ESP_LOGE("State:", "SET_ANGLE_CORRECTION_THETA\n");
				#endif
				param1 = received_param;
				current_state = SET_ANGLE_CORRECTION_SPEED;
			}else{
				param2 = received_param;
				#ifdef DEBUG
					ESP_LOGE("State:", "SET_ANGLE_CORRECTION_SPEED ang: %d em sp:%d %s\n", param1, param2, angle[wheels_direction]);
				#endif
				// implementar robo autonomo sentido horario com wheel direction
				parser_motor_package.packetID++;
				parser_motor_package.theta = param1; // Left speed
				parser_motor_package.speed_r = param2; // right speed
				parser_motor_package.speed_l = param2; // left speed
				parser_motor_package.wheels_direction = wheels_direction & 1; // Direction
				parser_motor_package.control_type = 0; // Old or new control type
        parser_motor_package.rotation_direction = rotation_direction >> 1;
				writeMotorPackage(&parser_motor_package);
				xTaskNotifyGive(motorTaskHandler);
				current_state = START;
			}
			break;

		case SET_PID_KP:
			if (current_state == SET_PID_KP){
				//ESP_LOGE("State:", "SET_PID_KP\n");
				++param1;
				kp = kp<<8 | received_param;
				if (param1==4){
					current_state = SET_PID_KI;
					param1 = 0;
				}
			}
			else if (current_state == SET_PID_KI){
				++param1;
				//ESP_LOGE("State:", "SET_PID_KI: %d\n", aux);
				ki = ki<<8 | received_param;
				if (param1==4){
					current_state = SET_PID_KD;
					param1 = 0;
				}
			}else{
				// implementar update de PID
				// usando param1, 2 e o received
				++param1;
				kd = kd<<8 | received_param;
				if (param1==4){
					current_state = START;
					param1 = 0;
					#ifdef DEBUG
						ESP_LOGI("State:", "SET_PID: P:%f I:%f D:%f\n", *((float*)&kp), *((float*)&ki), *((float*)&kd));
					#endif
				}
				pid_controller.set_PID(*((float*)&kp), *((float*)&ki), *((float*)&kd));
			}
			break;

		default:
			// tratamento de exceção
			ESP_LOGE("PARSER ERROR", "Current State: %d F:%d S:%d WD:%d\n", current_state, first_operation, second_operation,wheels_direction);
			current_state = START;
			break;
	}
}
