#include "parser.h"
#include "definitions.h"
#include "esp_log.h"
#include "PIDController.h"
#include "state_machine_definitions.h"
static int wheels_direction = 0; 
static int param1 = 0;
static int param2 = 0;
static int current_state = START; 
static int first_operation = 0;
static int second_operation = 0;
static int sum_for_next_operation[4] = {0,MOTOR_VELOCITY_PARAM_1, SET_ANGLE_CORRECTION_THETA, SET_PID_KP};

// #define DEBUG

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
extern motorPackage motor_package;
extern controlPackage control_package;


void parser_params(uint8_t received_param){

	first_operation = current_state & 82; // 01010010
	// Bit mask for reducing the if comparassions

	switch (first_operation) {
		case START: // State 00000000
			second_operation = (received_param & 12)>>2; // Bit mask for reducing if comparassions in
			// sum_for_next_operation at position second_operation
			// this allows for the code to jump to next state  
			wheels_direction = received_param & 3; // Wheels orientation
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
				motor_package.direction = wheels_direction; // Direction
				motor_package.speed_l = param1; // Left speed
				motor_package.speed_r = param2; // right speed
				motor_package.control_type = 1; // Old or new control type
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
					ESP_LOGE("State:", "SET_ANGLE_CORRECTION_SPEED ang: %d em sp:%d %s\n",param1, param2, angle[wheels_direction]);
				#endif
				// implementar robo autonomo sentido horario com wheel direction
				current_state = START;
			}
			break;
		
		case SET_PID_KP:
			if (current_state == SET_PID_KP){
				//ESP_LOGE("State:", "SET_PID_KP\n");
				param1 = received_param;
				current_state = SET_PID_KI;
			}
			else if (current_state == SET_PID_KI){
				//ESP_LOGE("State:", "SET_PID_KI\n");
				param2 = received_param;
				current_state = SET_PID_KD;
			}else{
				#ifdef DEBUG 
					ESP_LOGE("State:", "SET_PID: P:%d I:%d D:%d\n",param1,param2,received_param);
				#endif
				// implementar update de PID
				// usando param1, 2 e o received
				pid_controller.set_PID(param1,param2,received_param);
				current_state = START;
			}
			break;

		default:
			// tratamento de exceção
			ESP_LOGE("PARSER ERROR", "Current State: %d F:%d S:%d WD:%d\n", current_state, first_operation, second_operation,wheels_direction);
			break;
	}
}