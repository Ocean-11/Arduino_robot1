
/*********************************************************************
* Arduino robot resources - digital IO
**********************************************************************/

#define ODO_RIGHT                   2   // Odometry pullup input 1:60 motor
#define ODO_LEFT                    3   // Odometry pullup input 1:60 motor
#define TURBINE_ODO                 5
#define DRIVE_RIGHT_PWM             6   // drive PWM
#define DRIVE_LEFT_PWM              7   // drive PWM
#define TORBINE_PWM                 8   // turbine PWM


#define SERVO_SPARE                 11

#define LEFT_FORWARD_EN             34 // direction pullup U need LOW to get forward 
#define LEFT_BACKWARD_EN            35 // direction pullup U need LOW to get backward
#define RIGHT_FORWARD_EN            22 // direction pullup U need LOW to get forward
#define RIGHT_BACKWARD_EN           23 // direction pullup U need LOW to get backward
#define LCD_RS                      24 //(Uno 8) 
#define LCD_E                       25 //(Uno 9)
#define LCD_D4                      26 //(Uno 4) 
#define LCD_D5                      27 //(Uno 5)
#define LCD_D6                      28 //(Uno 6)
#define LCD_D7                      29 //(Uno 7)
//#define US_RIGHT_PING               30 // ultrasonic output
//#define US_RIGHT_ECHO               31 // ultrasonic input
#define US_LEFT_PING                32
#define US_LEFT_ECHO                33

/*********************************************************************
* Arduino robot resources - analog IO
**********************************************************************/

#define LCD_BUTTONS                 0 // (Uno A0)
#define DRIVE_RIGHT_CUR             1
#define DRIVE_LEFT_CUR              2
#define CLIFF_RIGHT                 3 // Sharp IR Right
#define CLIFF_LEFT                  4 
#define CLIFF_SPARE1                5
#define CLIFF_SPARE2                6
#define TURBINE_CUR                 7 // turbine current
