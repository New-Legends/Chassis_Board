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
#define KEY_PRESSED_CHASSIS_TOP     'F'

//底盘摇摆  单击C
#define KEY_PRESSED_CHASSIS_SWING   'C'

//底盘45度角 单击V
#define KEY_PRESSED_CHASSIS_PISA    'V'

//底盘超级电容加速 单击SHIFT !代表shift
#define KEY_PRESSED_CHASSIS_SUPER_CAP    '!'

//底盘前后左右控制按键
#define KEY_PRESSED_CHASSIS_FRONT  'W'
#define KEY_PRESSED_CHASSIS_BACK   'S'
#define KEY_PRESSED_CHASSIS_LEFT   'A'
#define KEY_PRESSED_CHASSIS_RIGHT  'D'


#endif