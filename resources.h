
/*********************************************************************
* Arduino robot resources - digital IO
**********************************************************************/

#define ODO_RIGHT                   2   // Odometry pullup input 1:60 motor
#define ODO_LEFT                    3   // Odometry pullup input 1:60 motor
#define DRIVE_RIGHT_FW              6   // drive PWM
#define DRIVE_LEFT_FW               7   // drive PWM

#define HVLP_SERVO                  11

#define DRIVE_RIGHT_DIR             22
#define DRIVE_LEFT_DIR              23
#define LCD_RS                      24 //(Uno 8) 
#define LCD_E                       25 //(Uno 9)
#define LCD_D4                      26 //(Uno 4) 
#define LCD_D5                      27 //(Uno 5)
#define LCD_D6                      28 //(Uno 6)
#define LCD_D7                      29 //(Uno 7)
#define US1_PING                    30
#define US1_ECHO                    31
#define US2_PING                    32
#define US2_ECHO                    33

/*********************************************************************
* Arduino robot resources - analog IO
**********************************************************************/

#define LCD_BUTTONS                 0 // (Uno A0)
#define DRIVE_RIGHT_CUR             1
#define DRIVE_LEFT_CUR              2
#define CLIFF_RIGHT                 3   // Sharp IR Right
#define CLIFF_LEFT                  4   // Sharp IR Left
