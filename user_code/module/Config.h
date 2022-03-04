#ifndef CONFIG_H
#define CONFIG_H

/*----------------------底盘---------------------------*/
//底盘动力电机无电流输出
#define CHASSIS_MOTIVE_MOTOR_NO_CURRENT 0

//底盘舵向电机无电流输出
#define CHASSIS_RUDDER_MOTOR_NO_CURRENT 0


/*---------------------通信-----------------------------*/
//底盘遥控器是否开启 正常上下板通信是关闭状态的
#define CHASSIS_REMOTE_OPEN 0


/*---------------------按键---------------------------*/
//底盘小陀螺 单击F
#define KEY_PRESSED_CHASSIS_TOP     KEY_PRESSED_OFFSET_F

//底盘摇摆  单击C
#define KEY_PRESSED_CHASSIS_SWING   KEY_PRESSED_OFFSET_C

//底盘45度角 单击V
#define KEY_PRESSED_CHASSIS_PISA    KEY_PRESSED_OFFSET_V

//底盘前后左右控制按键
#define KEY_PRESSED_CHASSIS_FRONT  KEY_PRESSED_OFFSET_W
#define KEY_PRESSED_CHASSIS_BACK   KEY_PRESSED_OFFSET_S
#define KEY_PRESSED_CHASSIS_LEFT   KEY_PRESSED_OFFSET_A
#define KEY_PRESSED_CHASSIS_RIGHT  KEY_PRESSED_OFFSET_D


#endif