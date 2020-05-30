#include "esp_log.h"
// #include "freertos/task.h"

#include "PIDController.h"
#include "parser.h"
#include "definitions.h"

static int op_code = 0;
static int operation_arguments;
static uint32_t *pid_pointer;
static float kp=0, ki=0, kd=0; 

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

void parser_params(uint8_t* received_param){

	op_code = (*received_param)  >> 4; // XXXX 0000 -> 0000 XXXX
	operation_arguments = (*received_param) & 15; //0000 XXXX

	switch (op_code) {

		case SET_SPIN_CODE:
			++parser_motor_package.packetID;
			// 0001 = 1 -> esquerda frente, direita trás = horário
			// 0010 = 2 -> esquerda trás, direita frente = anti-horário
			// Setamos mas não usamos...
			// parser_motor_package.rotation_direction = operation_arguments == 1 ? HORARIO : ANTIHORARIO;
			parser_motor_package.wheels_direction = operation_arguments; // Direction
			parser_motor_package.control_type = SPIN;
			parser_motor_package.speed_l = *(received_param + 1);

			writeMotorPackage(&parser_motor_package);
			xTaskNotifyGive(motorTaskHandler);

			break;

		case SET_MOTOR_CODE: // State 00000000

			// MOTOR FORCE YEAH
			++parser_motor_package.packetID;
			parser_motor_package.wheels_direction = operation_arguments; // Direction
			parser_motor_package.speed_l = *(received_param + 1); // Left speed
			parser_motor_package.speed_r = *(received_param + 2); // right speed
			parser_motor_package.control_type = OLD_CONTROL; 

			#ifdef DEBUG
				ESP_LOGE("State:", "Set Motor Code L:%d R:%d WHEELSDIRECTION:%d\n",
					parser_motor_package.speed_l,
					parser_motor_package.speed_r,
					parser_motor_package.wheels_direction
				);
			#endif

			writeMotorPackage(&parser_motor_package);
			xTaskNotifyGive(motorTaskHandler);

			break;
		case SET_ANGLE_CORRECTION_CODE://> 7 && < 15:

			// implementar robo autonomo sentido horario com wheel direction
			++parser_motor_package.packetID;
			parser_motor_package.theta = *(received_param + 1); // Left speed
			parser_motor_package.speed_r = *(received_param + 2); // right speed
			parser_motor_package.speed_l = *(received_param + 2); // left speed
			parser_motor_package.wheels_direction = operation_arguments; // Direction
			parser_motor_package.control_type = NEW_CONTROL;

			#ifdef DEBUG
				ESP_LOGE("State:", "SET_ANGLE_CORRECTION_SPEED ang: %d em sp:%d %s\n", 
					parser_motor_package.theta,
					parser_motor_package.speed_r,
					angle[parser_motor_package.wheels_direction]
				);
			#endif

   			//parser_motor_package.rotation_direction = rotation_direction >> 1;
			writeMotorPackage(&parser_motor_package);
			xTaskNotifyGive(motorTaskHandler);
					
			break;

		case SET_PID_CODE:
			
			pid_pointer = (uint32_t*) (received_param+1);
			kp = *((float*) (pid_pointer) );
			ki = *((float*) (pid_pointer+1) );
			kd = *((float*) (pid_pointer+2) );

			#ifdef DEBUG
				ESP_LOGI("State:", "SET_PID: P:%f I:%f D:%f\n", 
					kp,
					ki,
					kd
				);
			#endif

			pid_controller.set_PID( kp, ki, kd);

			break;
		default:
			// tratamento de exceção
			ESP_LOGE("PARSER ERROR", "Op_Code:%d Arguments:%d", op_code, operation_arguments);
			break;
	}
}