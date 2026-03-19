#ifndef _REMOTE_H_
#define _REMOTE_H_

#include "Task_Init.h"
#include <stdbool.h>
#include "PID.h"
#include "VESC.h"

#define PI 3.14159265359f
#define MAX_VELOCITY 10.0f	  // 底盘最大速度
#define MAX_OMEGA PI*10	 	 //最大角速度
#define R 0.457f	 	//整车半径
#define WHEEL_RADIUS 0.075f  //轮的半径


//电机参数
typedef struct{
	PID_EREOR_TypeDef PID;
	VESC_t steering;
	uint8_t deadband;//死区控制（有效遏制电机抖动）
}Motor_param;


typedef enum {
     STP,//自动模式下的急停
     STOP,//遥控模式下的急停
     REMOTE,
	   CHOOSE,
}Positon_label;

//任务
extern TaskHandle_t Move_Remote_Handle;
extern TaskHandle_t Remote_Handle;

//模式
extern Positon_label MODE;

//任务函数
void Remote(void *pvParameters);
void Move_Remote(void *pvParameters);

bool is_remote_active(void);
#endif
