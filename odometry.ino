
/*******************************************************************************
* Calculate Velocity - rewrite as a class !!!!!
*******************************************************************************/ 

float calculate_right_vel(float prev_velocity)
{ 
  static unsigned long prev_odo_ticks = 0;
  static unsigned long prev_odo_time_us = 0;
  static int print_counter = 0;
  unsigned long odo_ticks_diff = 0;
  unsigned long odo_time_diff_us = 0;
  double motor_velocity_double_mm;
  double motor_velocity_double_cm;
  int motor_velocity_10;
  float motor_velocity_float;
  
  // copy ISR values to avoid corruption - the following lines should not be seperated!
  unsigned long local_odo_ticks = odo_ticks_r;
  unsigned long local_odo_time_us = odo_time_us_r;   
  if (odo_ticks_r != local_odo_ticks)
  {
    // ISR has corrupted the data -> do not update velocity
    Serial.println("Right ISR - corrupted data");
    return prev_velocity;
  }  

  // calculate odo ticks
  if (local_odo_ticks>= prev_odo_ticks)
  {
    odo_ticks_diff = local_odo_ticks - prev_odo_ticks;    
  }
  else
  {
    // wrap around - update ticks & time and return
    Serial.println("righgt ticks: wrap around");
    prev_odo_ticks = local_odo_ticks;
    prev_odo_time_us = local_odo_time_us;
    return prev_velocity; // to be handled .... 
  }

  if (local_odo_time_us>= prev_odo_time_us)
    odo_time_diff_us = local_odo_time_us - prev_odo_time_us;
  else
  {
    // wrap around
    return prev_velocity; // to be handled ....   
  }
  
  // Calculate velocity
  if (micros()-local_odo_time_us>ODO_NO_MOVEMENT)
  {
    // No new pulses within the last 500ms -> zero velocity
    motor_velocity_float = 0.0;
  }
  else
  {
    if (odo_ticks_diff<MIN_ODO_TICKS)
    {
      // Not enough new ticks -> return last measured velocity
      motor_velocity_float = prev_velocity;
    }
    else
    {
      // calculate new velocity
      //motor_velocity_double = (odo_ticks_diff/TICKS_FOR_ROTATION*3.1415*DIAMETER)/(odo_time_diff_us/1000000)
      //motor_velocity_double_mm = double(1000000.0 * double(odo_ticks_diff)*3.1415*DIAMETER)/double(odo_time_diff_us)/double(TICKS_FOR_ROTATION)/10.0; // maybe due to the 5ms ISR delay?
      motor_velocity_double_mm = double(1000000.0 * double(odo_ticks_diff)*3.1415*DIAMETER)/double(odo_time_diff_us)/double(TICKS_FOR_ROTATION)/2.0;  // Try with the new ISR
      motor_velocity_double_cm = motor_velocity_double_mm/10.0;
      //motor_velocity_10 = int(motor_velocity_double*10);
      motor_velocity_float = float(motor_velocity_double_cm);

      // Update previous values
      prev_odo_ticks = local_odo_ticks;
      prev_odo_time_us = local_odo_time_us;
    }    
  }
  
  print_counter++;
  if (print_counter>100)
  {
    print_counter = 0;
    Serial.print("right ticks: ");
    Serial.println(odo_ticks_diff);
    Serial.print("time diff us= ");
    Serial.println(odo_time_diff_us);
    Serial.print("motor vel= ");
    Serial.println(motor_velocity_float);
  } 
  return motor_velocity_float;
}


float calculate_left_vel(float prev_velocity)
{ 
  static unsigned long prev_odo_ticks = 0;
  static unsigned long prev_odo_time_us = 0;
  static int print_counter = 0;
  unsigned long odo_ticks_diff = 0;
  unsigned long odo_time_diff_us = 0;
  double motor_velocity_double_mm;
  double motor_velocity_double_cm;
  int motor_velocity_10;
  float motor_velocity_float;
  
  // copy ISR values to avoid corruption - the following lines should not be seperated!
  unsigned long local_odo_ticks = odo_ticks_l;
  unsigned long local_odo_time_us = odo_time_us_l;   
  if (odo_ticks_l != local_odo_ticks)
  {
    // ISR has corrupted the data -> do not update velocity
    Serial.println("Left ISR - corrupted data");
    return prev_velocity;
  }  

  // calculate odo ticks
  if (local_odo_ticks>= prev_odo_ticks)
  {
    odo_ticks_diff = local_odo_ticks - prev_odo_ticks;    
  }
  else
  {
    // wrap around - update ticks & time and return
    Serial.println("left ticks: wrap around");
    prev_odo_ticks = local_odo_ticks;
    prev_odo_time_us = local_odo_time_us;
    return prev_velocity; // to be handled .... 
  }

  if (local_odo_time_us>= prev_odo_time_us)
    odo_time_diff_us = local_odo_time_us - prev_odo_time_us;
  else
  {
    // wrap around
    return prev_velocity; // to be handled ....   
  }
  
  // Calculate velocity
  if (micros()-local_odo_time_us>ODO_NO_MOVEMENT)
  {
    // No new pulses within the last 500ms -> zero velocity
    motor_velocity_float = 0.0;
  }
  else
  {
    if (odo_ticks_diff<MIN_ODO_TICKS)
    {
      // Not enough new ticks -> return last measured velocity
      motor_velocity_float = prev_velocity;
    }
    else
    {
      // calculate new velocity
      //motor_velocity_double = (odo_ticks_diff/TICKS_FOR_ROTATION*3.1415*DIAMETER)/(odo_time_diff_us/1000000)
      //motor_velocity_double_mm = double(1000000.0 * double(odo_ticks_diff)*3.1415*DIAMETER)/double(odo_time_diff_us)/double(TICKS_FOR_ROTATION)/10.0; // maybe due to the 5ms ISR delay?
      motor_velocity_double_mm = double(1000000.0 * double(odo_ticks_diff)*3.1415*DIAMETER)/double(odo_time_diff_us)/double(TICKS_FOR_ROTATION)/2.0;  // Try with the new ISR
      motor_velocity_double_cm = motor_velocity_double_mm/10.0;
      //motor_velocity_10 = int(motor_velocity_double*10);
      motor_velocity_float = float(motor_velocity_double_cm);

      // Update previous values
      prev_odo_ticks = local_odo_ticks;
      prev_odo_time_us = local_odo_time_us;
    }    
  }
  
   print_counter++;
  if (print_counter>100)
  {
    print_counter = 0;
    Serial.print("left ticks: ");
    Serial.println(odo_ticks_diff);
    Serial.print("time diff us= ");
    Serial.println(odo_time_diff_us);
    Serial.print("motor vel= ");
    Serial.println(motor_velocity_float);
  } 
  return motor_velocity_float;
}

/*******************************************************************************
* Odometry ISR based on SW timer 
*******************************************************************************/

void odo_ISR_right()
{

  odo_ticks_r++;
  odo_time_us_r = micros();  //try to avoid functions !!
  //delay(5); // check if delay needed!! - detach the interrupt instead and attach again after 5ms interrupt ..
  detachInterrupt(digitalPinToInterrupt(ODO_RIGHT));
  //is_right_ISR_en = false;
  //Timer1.start();  // Start Timer1 that would re-attach the ISR
  Timer3.start();  // Start Timer1 that would re-attach the ISR
  
}

/*******************************************************************************
* Odometry ISR based on SW timer 
*******************************************************************************/

void odo_ISR_left()
{

  odo_ticks_l++;
  odo_time_us_l = micros();  //try to avoid functions !!
  //delay(5); // check if delay needed!! - detach the interrupt instead and attach again after 5ms interrupt ..
  detachInterrupt(digitalPinToInterrupt(ODO_LEFT));
  //is_left_ISR_en = false;
  //Timer1.start();  // Start Timer1 that would re-attach the ISR
  Timer3.start();  // Start Timer1 that would re-attach the ISR
  
}

/*******************************************************************************
* Enable odo ISR 
*******************************************************************************/

void enable_odo_ISR()
{
  // enable interruots when both left and right debounce have elapsed
  attachInterrupt(digitalPinToInterrupt(ODO_RIGHT), odo_ISR_right, RISING); 
  attachInterrupt(digitalPinToInterrupt(ODO_LEFT), odo_ISR_left, RISING); 
}


/*******************************************************************************
* Calculate the odometry (taken from TB3)
*******************************************************************************/
/*bool calc_odometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}
*/
