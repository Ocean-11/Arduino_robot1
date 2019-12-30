
/*********************************************************************
* Arduino robot
**********************************************************************/



/*********************************************************************
* Arduino Robot
* 
**********************************************************************/

#include "resources.h"
#include "odometry.h"

//#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

#include <Servo.h>
#include <LiquidCrystal.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

// ROS
//ros::NodeHandle nh; // keep as a note to minimize memory (1Kbyte)

//#include <math.h>

// Firmware version
#define FIRMWARE_VER "0.0.1"

#define OPT_TIME_ISR_
#define DIRECT_MOTOR_

// Tasks frequencies
#define CONTROL_MOTOR_SPEED_FREQUENCY          50   //hz
#define LCD_FREQUENCY                          10   //hz

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define MAX_REQ_VEL             25
#define REQ_VEL_STEP            1
#define MAX_SERVO_DELAY         2000
#define SERVO_DELAY_STEP        200

// HVLP parameters
#define HVLP_CLOSE_POS                   (uint32_t)120 // closed position angle (deg)
#define HVLP_OPEN_POS                    (uint32_t)80 // open position angle (deg) - corresponds to 180deg
#define HVLP_MAX                         (uint32_t)170
#define HVLP_MIN                         (uint32_t)30
#define HVLP_ANGLE_STEP                   5 

// Menu places
#define MENU_REQ_VELOCITY        0
#define MENU_OPEN_ANGLE          1
#define MENU_CLOSE_ANGLE         2
#define MENU_SERVO_DELAY         3
#define MENU_HVLP_EN             4
#define MENU_FEEDER_EN           5
#define MAX_MENU_PLACES          6

// Buttons
#define NO_BUTTON                0
#define RIGHT_BUTTON             1
#define LEFT_BUTTON              2
#define UP_BUTTON                3
#define DOWN_BUTTON              4
#define SELECT_BUTTON            5

/*******************************************************************************
* SoftwareTimer
*******************************************************************************/
static uint32_t tTime[10];

// Control
#define MAX_PWM_CHANGE       20
#define MAX_PWM              254
#define MIN_PWM              0

// Function prototypes
float calculateVelocity(void);
void menu_handler_on(int velocity);
void print_idle_screen(void); 
void print_on_screen(void); 
void get_button(void);
void menu_handler(void);
void update_screen(int);
//void motor_control(int, int); 
float calculate_right_vel(float);
float calculate_left_vel(float);
void odo_ISR_right(void);  
void odo_ISR_left(void);  
//void motor_control(float, float); 

/*******************************************************************************
* Declaration for GUI & menu
*******************************************************************************/
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // initialize the library with the numbers of the interface pins
int menu_place_id = MENU_REQ_VELOCITY;
int current_button = NO_BUTTON;

bool is_pressed = false;            // remove this parameter !!!!!!!!!!!!!!!!!!!!!!

/*******************************************************************************
* Declaration for time optimized velocity ISR measurement
*******************************************************************************/
volatile unsigned long odo_ticks_r = 0;
volatile unsigned long odo_time_us_r = 0;
bool is_right_ISR_en = true;
volatile unsigned long odo_ticks_l = 0;
volatile unsigned long odo_time_us_l = 0;
bool is_left_ISR_en = true;

/*******************************************************************************
* Declaration for HVLP servo
*******************************************************************************/
Servo myservo;  // create servo object to control a servo

//Servo HVLP_servo;  // create servo object to control the HVLP servo
uint8_t servo_angle = 0;
int32_t hvlp_close_angle = HVLP_CLOSE_POS; 
int32_t hvlp_open_angle = HVLP_CLOSE_POS;  

/*******************************************************************************
* Declaration for jig parameters
*******************************************************************************/
bool is_feeder_active = false;      // Feeder starts inactive
int req_velocity = 15;              // required feeder velocity [cm/sec]
int open_angle = HVLP_OPEN_POS;     // HVLP active position
int close_angle = HVLP_CLOSE_POS;   // HVLP idle position
int servo_delay = 1000;             // wait 1000ms brfore activating the HVLP
bool is_feeder_enabled = true;      // Feeder is enabled 
bool is_hvlp_enabled = true;        // HVLP is enabled

/*******************************************************************************
* Declaration for velocity ISR measurement
*******************************************************************************/
int odo_intr = 0;
int motor_rpm=0;
float dTime = 0;
unsigned long time_for_rotation;

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

/*******************************************************************************
* Declaration for motor controller
*******************************************************************************/



/*#include <ros.h>
#include <std_msgs/String.h>

// Includes from TB3:
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h> // RAN
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Servo.h>
#include <math.h>
#include "robot_config.h"

// Firmware version
#define FIRMWARE_VER "0.0.1"

// Tasks frequencies
#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define PAYLOAD_SPEED_FREQUENCY                30   //hz
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

// Drive
#define WHEEL_NUM                        2
#define LEFT                             0
#define RIGHT                            1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

// define PWM pins
#define PWM_PIN_PUMP                     3  // paint pump pin
#define PWM_PIN_BUFFER                   6  // buffer pin
#define BUFFER_OFF_FILTER                3  // [seconds]
#define PUMP_IGNITION_TIME               5  // [seconds]
#define PUMP_CYCLE_ON_TIME               2  // [seconds]
#define PUMP_CYCLE_OFF_TIME              3  // [seconds] 


// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

// Function prototypes
void updateTime(void);
*/

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
/*ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;*/

/*******************************************************************************
* ROS Parameter
*******************************************************************************/


/*******************************************************************************
* Subscriber
*******************************************************************************/


/*******************************************************************************
* Publisher
*******************************************************************************/
/*
// Chatter example 
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// Odometry
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Battey state
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);*/

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
/*static uint32_t tTime[10];

char hello[13] = "hello world!";

// Left wheel controlled by pins 2,3,5
// Right wheel cpntrolled by pins 9,11,10


#define RIGHT_ODOMETRY 9
#define RIGHT_PWMF 11
#define RIGHT_PWMR 10
//#define RIGHT_ALARMF A2
//#define RIGHT_ALARMR A3


#define LEFT_ODOMETRY 2
#define LEFT_PWMF 3
#define LEFT_PWMR 5
//#define LEFT_ALARMF A0
//#define LEFT_ALARMR A1



//#define GEAR_RATIO  82
//#define DIAMETER 150
//#define PULSES_2_MM (DIAMETER * 3.1415) / GEAR_RATIO

int calculateDistance(int pulses);

int right_pulse = 0;
int prev_right_pulse = 0;
int right_odo_counter = 0;
float right_prev_time =0;
float right_time_taken=0;
float right_rpm=0;
float right_V;
int left_pulse = 0;
int prev_left_pulse = 0;
int left_odo_counter = 0;
float left_prev_time =0;
float left_time_taken=0;
float left_rpm=0;
float left_V;

*/
