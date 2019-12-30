#include "Arduino_robot.h" 
#include <TimerThree.h>
//#include <TimerOne.h>  // Requires installing TimerOne library

/*******************************************************************************
* motor control class 
*******************************************************************************/

class motor_control
{
  // Class member variables (initialized at startup)
  private:
    unsigned long t;
    int prev_pwm;
    float prev_velocity;
    float prev_diff;  
    unsigned long prev_time;
    int required_pwm;
  
  // Constructor
  public:
    motor_control()
    {
      t = millis();
      prev_pwm=0;
      prev_velocity = 0.0;
      prev_diff = 0.0;
      prev_time = 0;
      required_pwm = 0;
    }

    void update(float required_velocity, float current_velocity, float P, float D, float I)    
    {
      int delta_pwm;
      float vel_diff;  
      float differntial;
      float delta_P_pwm, delta_D_pwm, delta_I_pwm;  
      
      // P part
      delta_P_pwm = (required_velocity - current_velocity) * P;
  
      // D part
      delta_D_pwm = (current_velocity - prev_velocity)*1000*D/float(t-prev_time);
    
      // I part
      delta_I_pwm = 0.0;
    
      // Calculate new PWM
      delta_pwm = int(delta_P_pwm + delta_D_pwm);
    
      // Hard limit the max pwm change
      if (delta_pwm > MAX_PWM_CHANGE)
        delta_pwm = MAX_PWM_CHANGE;
      else if (delta_pwm < -MAX_PWM_CHANGE)
        delta_pwm = -MAX_PWM_CHANGE;
  
      // update the pwm
      required_pwm += delta_pwm;
           
      // Limit the actual pwm to be applied
      if (required_pwm>MAX_PWM)
        required_pwm = MAX_PWM;
      else if (required_pwm<MIN_PWM)
        required_pwm=MIN_PWM;
  
      // take into account the min PWM in which the wheels start to spin in the air!!!!!!!
    
      // Update the PWM
      analogWrite(DRIVE_LEFT_FW,required_pwm);
      //analogWrite(MOTOR_PWMF,120);
  
  
      // Update parameters
      prev_time = t;
      prev_velocity = current_velocity; 
  
      // Debug strings
      Serial.print("Left Vel: ");
      Serial.print(current_velocity);
      Serial.print(" PWM: ");
      Serial.println(required_pwm);
  
    } 
};

motor_control motor_control_r = motor_control();
motor_control motor_control_l = motor_control();

void setup() {

  // Init odometry pins & timer3
  pinMode(ODO_RIGHT, INPUT_PULLUP);  // use pullup also for the no gear motor !! 
  pinMode(ODO_LEFT, INPUT_PULLUP);  // make pin 1 an input (odometry)
  attachInterrupt(digitalPinToInterrupt(ODO_RIGHT), odo_ISR_right, RISING);
  attachInterrupt(digitalPinToInterrupt(ODO_LEFT), odo_ISR_left, RISING);   
  Timer3.initialize(5000);
  Timer3.attachInterrupt(enable_odo_ISR);

  // Init drive PWM timer4 to 31250
  TCCR4B=(TCCR2B&0xF8) | 1;
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;
 
         
  // Init debug port
  Serial.begin(9600);

  // Setup motor PWM
  //analogWrite(DRIVE_RIGHT_FW,120); //do 50% PWM on pin 11 at the frequency set in TCCR2B(right wheel) 
  //analogWrite(DRIVE_LEFT_FW,120); 
 
  // Setup the LCD  
  lcd.begin(16, 2);
  //lcd.setCursor(0,1);
  //lcd.print("Hello         ");
 
}

/*******************************************************************************
* Loop function
*******************************************************************************/

void loop() 
{
  uint32_t t = millis();
  static float current_right_vel = 0.0; 
  static float current_left_vel = 0.0; 
    
  // Call RPM controller
  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {  
    // Poll motor odometry
    //current_velocity = calculateVelocity();
    current_right_vel = calculate_right_vel(current_right_vel);
    current_left_vel = calculate_left_vel(current_left_vel);
        
    // Activate motor control
    //motor_control_r.update(float(req_velocity), current_right_vel, 5.0, 0.0, 0.0); 
    //motor_control_l.update(float(req_velocity), current_left_vel, 5.0, 0.0, 0.0); 

    //PID_motor_control_r(float(req_velocity), current_right_vel, 5.0, 0.0, 0.0); 
    //PID_motor_control_l(float(req_velocity), current_left_vel, 5.0, 0.0, 0.0); 
    
    if (is_feeder_active)
    {
      PID_motor_control_r(float(req_velocity), current_right_vel, 5.0, 0.0, 0.0); 
      PID_motor_control_l(float(req_velocity), current_left_vel, 5.0, 0.0, 0.0); 
    }
    else
    {
      PID_motor_control_r(0.0, current_right_vel, 5.0, 0.0, 0.0); 
      PID_motor_control_l(0.0, current_left_vel, 5.0, 0.0, 0.0);
    }
     
    tTime[0] = t;
  }  

  // Handle the menu
  menu_handler();

  // Update the LCD

  if ((t-tTime[1]) >= (1000 / LCD_FREQUENCY))
  {
     update_screen(current_right_vel);
     tTime[1] = t;
  }   
}

/*******************************************************************************
* Right PID motor control 
*******************************************************************************/

void PID_motor_control_r(float required_velocity, float current_velocity, float P, float D, float I)
{

  unsigned long t=millis();
  static int prev_pwm = 0;
  static float prev_velocity = 0;
  static float prev_diff = 0;  
  static unsigned long prev_time=0;
  static int required_pwm=0;
  int delta_pwm = 0;
  float vel_diff;  
  float differntial = 0;
  float delta_P_pwm, delta_D_pwm, delta_I_pwm;  
   
  // P part
  delta_P_pwm = (required_velocity - current_velocity) * P;

  // D part
  delta_D_pwm = (current_velocity - prev_velocity)*1000*D/float(t-prev_time);

  // I part
  delta_I_pwm = 0.0;

  // Calculate new PWM
  delta_pwm = int(delta_P_pwm + delta_D_pwm);

  // Hard limit the max pwm change
  if (delta_pwm > MAX_PWM_CHANGE)
    delta_pwm = MAX_PWM_CHANGE;
  else if (delta_pwm < -MAX_PWM_CHANGE)
    delta_pwm = -MAX_PWM_CHANGE;

  // update the pwm
  required_pwm += delta_pwm;
         
  // Limit the actual pwm to be applied
  if (required_pwm>MAX_PWM)
    required_pwm = MAX_PWM;
  else if (required_pwm<MIN_PWM)
    required_pwm=MIN_PWM;

  // take into account the min PWM in which the wheels start to spin in the air!!!!!!!

  // Update the PWM
  analogWrite(DRIVE_RIGHT_FW,required_pwm);
  //analogWrite(MOTOR_PWMF,120);


    // Update parameters
  prev_time = t;
  prev_velocity = current_velocity; 

  // Debug strings
  Serial.print("Right Vel: ");
  Serial.print(current_velocity);
  Serial.print(" PWM: ");
  Serial.println(required_pwm);

}

/*******************************************************************************
* Left PID motor control 
*******************************************************************************/

void PID_motor_control_l(float required_velocity, float current_velocity, float P, float D, float I)
{

  unsigned long t=millis();
  static int prev_pwm = 0;
  static float prev_velocity = 0;
  static float prev_diff = 0;  
  static unsigned long prev_time=0;
  static int required_pwm=0;
  int delta_pwm = 0;
  float vel_diff;  
  float differntial = 0;
  float delta_P_pwm, delta_D_pwm, delta_I_pwm;  
   
  // P part
  delta_P_pwm = (required_velocity - current_velocity) * P;

  // D part
  delta_D_pwm = (current_velocity - prev_velocity)*1000*D/float(t-prev_time);

  // I part
  delta_I_pwm = 0.0;

  // Calculate new PWM
  delta_pwm = int(delta_P_pwm + delta_D_pwm);

  // Hard limit the max pwm change
  if (delta_pwm > MAX_PWM_CHANGE)
    delta_pwm = MAX_PWM_CHANGE;
  else if (delta_pwm < -MAX_PWM_CHANGE)
    delta_pwm = -MAX_PWM_CHANGE;

  // update the pwm
  required_pwm += delta_pwm;
         
  // Limit the actual pwm to be applied
  if (required_pwm>MAX_PWM)
    required_pwm = MAX_PWM;
  else if (required_pwm<MIN_PWM)
    required_pwm=MIN_PWM;

  // take into account the min PWM in which the wheels start to spin in the air!!!!!!!

  // Update the PWM
  analogWrite(DRIVE_LEFT_FW,required_pwm);
  //analogWrite(MOTOR_PWMF,120);


    // Update parameters
  prev_time = t;
  prev_velocity = current_velocity; 

  // Debug strings
  Serial.print("Left Vel: ");
  Serial.print(current_velocity);
  Serial.print(" PWM: ");
  Serial.println(required_pwm);

}




/*
class wheel_control
{
  float prev_velocity = 0.0;

  void begin()
  {
  }
  }
}*/
