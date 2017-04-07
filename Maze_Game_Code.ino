#include <Servo.h>                                                    // Call the servo library; Pins 9 and 10 lose PWN capacity                   
#include <Wire.h>                                                     // Call the wire library; Pins A4 and A5 lose their analog-to-digital capabilities
                                                                      // Declare and assign program constants
const int joystick_input_blue = A1,                                   // Assign pin numbers
          joystick_input_yellow = A0, 
          servo_pin_left = 6,
          servo_pin_right = 7,
          auto_pin = 50,
          auto_stop_pin = 52,
          main_control_pin = 46,
          IR_sensor = A2,
          Control_Trigger = 38,
          LED_Trigger = 44,
          lowerbound = 77,                                            // Assign constraints for the servos
          upperbound = 103,
          servo_center_left = 90,
          servo_center_right = 90,
          joystick_lower = 150,                                        // Assign joystick parameters
          joystick_upper = 900,
          joystick_right_center = 525,
          joystick_left_center = 525,
          joystick_deadzone = 50,
          mpu_lower = -9500,                                          // Assign MPU parameters
          mpu_upper = 9500,
          mpu_right_center = 0,
          mpu_left_center = 0,
          mpu_deadzone = 800,
          MPU_ADDR = 0x68,                                            // MPU Address
          delay_LED = 10000;                                          // Assign delay times
          
Servo servoLeft, servoRight;                                          // Create servo objects

int servo_val_left, servo_val_right, sensorState, lastState;          // Declare values
unsigned long counter_LED, counter, counter2;                         // Declare counter for delay times
boolean control_state = HIGH, control_current_state, control_last_state;   // For joystick/mpu control and Auto_Solver Stop
int16_t xAccel, yAccel, zAccel, temp, xGyro, yGyro, zGyro;            // Values for the MPU

/*========================================================================================================================================================================================*/

void setup() {                                                        // Begin setup
  
  Wire.begin();                                                       // Begin communication with the mpu‐6050
  Wire.beginTransmission(0x68);                                       // Write to the power management register
  Wire.write(0x6B);                                                   // Wake up the mpu‐6050
  Wire.write(0);                                                      // End communication with the mpu‐6050
  Wire.endTransmission(true);

  Serial.begin(9600);                                                   // Start serial communication; This assigns pins 0 and 1 for serial communication only
  while(!Serial);                                                     // Waits for serial to connect
  Serial.println("Serial communication established");                                                                     

  servoLeft.attach(servo_pin_left);                                   // Assign servo object to the correct pins
  servoRight.attach(servo_pin_right);
  Serial.println("Servos online");

  for (int i = 46; i < 54; i++)
    pinMode(i, INPUT_PULLUP);
  Serial.println("Pins established");
  
  Serial.print('\n');
  Serial.println("Setup complete");
  Serial.print('\n');
}                                                                     // End setup

/*========================================================================================================================================================================================*/

void loop() {                                                         // Begin loop
  Main_Control_Subroutine();
 
  if (digitalRead(auto_pin)== LOW)
    AutoSolver_Subroutine();
    
  control_current_state = debounce(control_last_state, main_control_pin);
  if (control_last_state == LOW && control_current_state == HIGH){
    control_state = !control_state;
  }
  control_last_state = control_current_state;
  
//  Serial.println(control_state);
  
}                                                                     // End loop

/*========================================================================================================================================================================================*/

boolean debounce(boolean last, int pin){
   boolean current = digitalRead(pin);
   if (last != current){
    delay(5);
    current = digitalRead(pin);
   }
   return current;
}

/*========================================================================================================================================================================================*/

void Main_Control_Subroutine(){ 
  if (control_state == LOW)
    Joystick_Subroutine();                                            // Call subroutine to run the joystick
  else if (control_state == HIGH)
    MPU_Subroutine();
}

/*========================================================================================================================================================================================*/

void Joystick_Subroutine(){                                           // Begin Joystick Subroutine
  servo_val_left = analogRead(joystick_input_blue);                   // Read the input values from the joystick
  servo_val_right = analogRead(joystick_input_yellow);

  if ((servo_val_left < joystick_left_center + joystick_deadzone) && (servo_val_left > joystick_left_center - joystick_deadzone))             // If the joystick is within the deadzone
    servoLeft.write(servo_center_left);                                                                                                       // Center the maze
  else if (servo_val_left >= joystick_left_center + joystick_deadzone){                                                                       // If the joystick is in the upper region
    servo_val_left = map(servo_val_left, joystick_left_center + joystick_deadzone, joystick_upper, servo_center_left, lowerbound);            // Map the joystick values to servo angles
    servo_val_left = constrain(servo_val_left, lowerbound, servo_center_left);
    servoLeft.write(servo_val_left);                                                                                                          // Write to the servo
  }
  else if (servo_val_left <= joystick_left_center - joystick_deadzone){                                                                       // If the joystick is in the lower region
    servo_val_left = map(servo_val_left, joystick_lower, joystick_left_center - joystick_deadzone, upperbound, servo_center_left);            // Map the joystick values to servo angles
    servo_val_left = constrain(servo_val_left, servo_center_left, upperbound);
    servoLeft.write(servo_val_left);                                                                                                          // Write to the servo
  }
  if ((servo_val_right < joystick_right_center + joystick_deadzone) && (servo_val_right > joystick_right_center - joystick_deadzone))         // If the joystick is within the deadzone
    servoRight.write(servo_center_right);                                                                                                     // Center the maze
  else if (servo_val_right >= joystick_right_center + joystick_deadzone){                                                                     // If the joystick is in the upper region
    servo_val_right = map(servo_val_right, joystick_right_center + joystick_deadzone, joystick_upper, servo_center_right, lowerbound);        // Map the joystick values to servo angles
    servo_val_right = constrain(servo_val_right, lowerbound, servo_center_right);
    servoRight.write(servo_val_right);                                                                                                        // Write to the servo
  }
  else if (servo_val_right <= joystick_right_center - joystick_deadzone){                                                                     // If the joystick is in the lower region
    servo_val_right = map(servo_val_right, joystick_lower, joystick_right_center - joystick_deadzone, upperbound, servo_center_right);        // Map the joystick values to servo angles
    servo_val_right = constrain(servo_val_right, servo_center_right, upperbound);
    servoRight.write(servo_val_right);                                                                                                        // Write to the servo
  }
}                                                                     // End Joystick Subroutine

/*========================================================================================================================================================================================*/

void MPU_Subroutine() {
  Wire.beginTransmission(MPU_ADDR);                                   // begin communication with the mpu‐6050
  Wire.write(0x3B);                                                   // write to the first data register
  Wire.endTransmission(false);                                        // end communication
  Wire.requestFrom(MPU_ADDR, 14, true);                               // request fourteen register reads from the mpu‐6050
  xAccel = Wire.read() << 8 | Wire.read();                            // read registers 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  yAccel = Wire.read() << 8 | Wire.read();                            // read registers 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  zAccel = Wire.read() << 8 | Wire.read();                            // read registers 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temp = Wire.read() << 8 | Wire.read();                              // read registers 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  xGyro = Wire.read() << 8 | Wire.read();                             // read registers 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  yGyro = Wire.read() << 8 | Wire.read();                             // read registers 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  zGyro = Wire.read() << 8 | Wire.read();                             // read registers 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  if ((xAccel < mpu_left_center + mpu_deadzone) && (xAccel > mpu_left_center - mpu_deadzone))                                 // If the joystick is within the deadzone
    servoLeft.write(servo_center_left);                                                                                         // Center the maze
  else if (xAccel >= mpu_left_center + mpu_deadzone){                                                                       // If the joystick is in the upper region
    servo_val_left = map(xAccel, mpu_left_center + mpu_deadzone, mpu_upper, servo_center_left, lowerbound);            // Map the joystick values to servo angles
    servo_val_left = constrain(servo_val_left, lowerbound, servo_center_left);
    servoLeft.write(servo_val_left);                                                                                                          // Write to the servo
  }
  else if (xAccel <= mpu_left_center - mpu_deadzone){                                                                       // If the joystick is in the lower region
    servo_val_left = map(xAccel, mpu_lower, mpu_left_center - mpu_deadzone, upperbound, servo_center_left);            // Map the joystick values to servo angles
    servo_val_left = constrain(servo_val_left, servo_center_left, upperbound);
    servoLeft.write(servo_val_left);                                                                                                          // Write to the servo
  }
  if ((yAccel < mpu_right_center + mpu_deadzone) && (yAccel > mpu_right_center - mpu_deadzone))         // If the joystick is within the deadzone
    servoRight.write(servo_center_right);                                                                                                     // Center the maze
  else if (yAccel >= mpu_right_center + mpu_deadzone){                                                                     // If the joystick is in the upper region
    servo_val_right = map(yAccel, mpu_right_center + mpu_deadzone, mpu_upper, servo_center_right, upperbound);         // Map the joystick values to servo angles
    servo_val_right = constrain(servo_val_right, servo_center_right, upperbound);
    servoRight.write(servo_val_right);                                                                                 // Write to the servo
  }
  else if (yAccel <= mpu_right_center - mpu_deadzone){                                                                     // If the joystick is in the lower region
    servo_val_right = map(yAccel, mpu_lower, mpu_right_center - mpu_deadzone, lowerbound, servo_center_right);        // Map the joystick values to servo angles
    servo_val_right = constrain(servo_val_right, lowerbound, servo_center_right);
    servoRight.write(servo_val_right); 
  }
}

/*========================================================================================================================================================================================*/

void AutoSolver_Subroutine(){                                         // Begin AutoSolver Subroutine 
  Serial.print('\n');
  Serial.println("AutoSolver Begin");
  Serial.print('\n');
  Serial.println("Executing first move");                             // Maze Contrains: 77, 103

  servoLeft.write(90);
  servoRight.write(90);
  delay(2000);

//  Auto_Stop = LOW;
  
 servoRight.write(100);//left 100
delay(2000);

servoLeft.write(100);//down 100
delay(500);

servoRight.write(80);//right 80
servoLeft.write(89);//up 89
delay(1000);

servoLeft.write(97);//down 100
delay(1420);

servoLeft.write(100);//down 100. hold marble.//////sometimes doesnt work
servoRight.write(85);//right 80. hold at in a corner
delay(2000);
Serial.println("test");


servoLeft.write(87);//up 88
servoRight.write(91);//left 90
delay(680);//previous was 680

servoRight.write(97);//left 95. put it in the gap
servoLeft.write(91);//down 91
delay(1000);

servoLeft.write(100);//down 95
delay(1000);

servoRight.write(95);//left 95
delay(1000);

servoLeft.write(85);//up 85
delay(1000);

servoLeft.write(95);//keep this code
servoLeft.write(85);//up 85. keep this code. marble rolls by itself through most of the past. on the left corner right now...
delay(5000);//keep this code

servoLeft.write(95);//down 95
servoRight.write(100);//left 95.
delay(1000);


delay(1000);


servoLeft.write(85);//up 85
delay(100);
servoRight.write(95);//left 95. ///holds it in a corner
delay(3000);

servoLeft.write(95);//down 100
servoRight.write(87);//right 87
delay(500);
servoRight.write(80);//right 80 //In gap
delay(1000);

servoLeft.write(95);//down 100 *
servoRight.write(85);//right 80 *
delay(1050);
servoLeft.write(85);//up 80 *
delay(1000);
servoRight.write(85);//right 80 *
delay(1000);


servoLeft.write(95);//down 100 *
delay(1000);
servoRight.write(95);//left 100 *
delay(1000);

servoRight.write(95);//left 100 *
delay(1000);



delay(1000);




servoRight.write(servo_center_right);//centers the maze
servoLeft.write(servo_center_left);  


  delay(2000);
  
  Serial.print('\n');
  Serial.println("AutoSolver Completed");
  Serial.print('\n');
}                                                                     // End AutoSolver Subroutine

/*========================================================================================================================================================================================*/

void LED_Subroutine1(){                                               // Trigger LED Program 1
  Serial.print('\n');
  Serial.println("Good job! You get a cookie");
  Serial.print('\n');
  }                                                                   // End LED Program 1
/*
    if (millis() > delay_print + counter){                              // This allows to have a delay function without interrupting the rest of the program
      Serial.print(servo_val_left);
      Serial.print(' ');
      Serial.println(servo_val_right);                             // This says "value1 value2" for the angles of the two servoes
      counter = millis();                                               // Update the counter
    }
    */
