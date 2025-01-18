#include "math.h"

// H-Bridge 1
const int pwmPin1 = 10;    // PWM pin for motor 1 (D9 on Arduino Nano)
const int dirPin1 = 9;    // Direction control pin for motor 1 (D8 on Arduino Nano)

// H-Bridge 2
const int pwmPin2 = 6;   // PWM pin for motor 2 (D10 on Arduino Nano)
const int dirPin2 = 5;    // Direction control pin for motor 2 (D7 on Arduino Nano)

const int wheel1 = A0;
const int wheel2 = A1;

byte b1 = 0;
byte b2 = 0;
byte b3 = 0;
byte b4 = 0;
byte message[4];
byte data[4];

bool received = false;
int state = 0 ;
float Speed ,Angle1 =0.0, Angle2 = 0.0;

float distance = 0.0;
int total_dist = 0;
unsigned long startTime;
unsigned long stopTime;
bool flag = true;
bool FF = false, FB = false;


int curr_time = 0;
int prev_time = 0;
int elapsedTimeSeconds = 0;

int savedPWM1 = 150;
int savedPWM2 = 0;

const float turnSpeedDegreesPerSecond = 156.0;

volatile int encoderCount1 = 0;  // For wheel 1
volatile int encoderCount2 = 0;  // For wheel 2
const float distancePerTick = 4.36;  
int encoderState1 = LOW;
int encoderState2 = LOW;
int prevEncoderState1 = LOW;
int prevEncoderState2 = LOW;
int countWheel1 = 0;
int countWheel2 = 0;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
const unsigned long debounceDelay = 5;


void setup() 
{

  Serial.begin(19200);
  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  

  pinMode(wheel1,INPUT);
  pinMode(wheel2,INPUT);

  stop();

}



void loop() 
{
  CheckMessage();
  updateEncoderCounts();


  if (received)
  {
    switch(state)
    {
      case 4:
      {
        if (b1 ==179)
        {
          forward();
          delay(1000);
          left(360);
          stop();
          b1 =0;
        }
        break;
      }

      case 1:
      {
        if (b1 == 96)
        {
          calibrate();
          stop();
          b2 = Speed;
          b3 = Speed;
          b4 = 0 ;
          message[0] = b1;
          message[1] = b2;
          message[2] = b3;
          message[3] = b4;
          Serial.write(message, 4);
          

          delay(500);
          b1 =97;
          b2 = 0;
          b3 = 0;
          b4 = 0; 
          message[0] = b1;
          message[1] = b2;
          message[2] = b3;
          message[3] = b4;
          Serial.write(message, 4);
          b1 =0 ;
          

        }
        else if(b1 == 97 )
        {
          b2 = 0;
          b3 = 0;
          b4 = 0; 
          message[0] = b1;
          message[1] = b2;
          message[2] = b3;
          message[3] = b4;
          Serial.write(message, 4);
          b1 = 0;
        }
        break;
       
      }
      case 2:
      {
        

        if (b1 == 161)
        { 
          // GetSpeed();
          b2 = 0;
          b3 = 0;
          b4 = 0 ;
          message[0] = b1;
          message[1] = b2;
          message[2] = b3;
          message[3] = b4;
          Serial.write(message, 4);

          if (int(data[3])==0 ||int(data[3])==1 )
          { 
            b1 = 162;
            b2 = 0;
            b3 = 0;
            b4 = 0 ;
            message[0] = b1;
            message[1] = b2;
            message[2] = b3;
            message[3] = b4;
            Serial.write(message, 4);

            if (flag == true)
            {
              startTime = millis();
              flag = false;
              distance = 0;
            }

            if(int(data[3]) ==0 )
            {
              
              if (FF == false )
              {
                startTime = millis();
                distance = 0;
              }
              FF = true;
              forward();
            }
            else
            {
              if (FB == false)
              {
                startTime = millis();
                distance = 0;
              }
              FB = true;
              reverse();
            }

            b1 = 163;
            b2 = Speed;
            b3 = Speed;
            b4 = 0 ;
            message[0] = b1;
            message[1] = b2;
            message[2] = b3;
            message[3] = b4;
            Serial.write(message, 4); 

       
            b1 = 164;
            GetDistance();
            b4 = 0 ;
            message[0] = b1;
            message[1] = b2;
            message[2] = b3;
            message[3] = b4;
            Serial.write(message, 4); 
            b1 = 0;

          }
          else if (int(data[3])==2 ||int(data[3])==3 )
          {

            b1 = 162;
            if(int(data[3]) ==2 )
            {
              left(Angle1+Angle2);
              distance =0;
              flag = true;
            }
            else
            {
              distance =0;
              flag = true;
              right(Angle1+Angle2);
            }
            GetDirection();
            b4 = int(data[3]) ;
            message[0] = b1;
            message[1] = b2;
            message[2] = b3;
            message[3] = b4;
            Serial.write(message, 4);



            

            b1 = 163;
            b2 = 0;
            b3 = 0;
            b4 = 0 ;
            message[0] = b1;
            message[1] = b2;
            message[2] = b3;
            message[3] = b4;
            Serial.write(message, 4);

            b1 = 164;
            b2 = 0 ;
            b3 = 0 ;
            b4 = 0 ;
            message[0] = b1;
            message[1] = b2;
            message[2] = b3;
            message[3] = b4;
            Serial.write(message, 4);
            b1 = 0;
          }

        }

        break;
      }
      
      case 3:
      {
        if (b1 == 228)
        {
          b2= 0;
          b3= 0;
          b4= 0;

          stop();
          message[0] = b1;
          message[1] = b2;
          message[2] = b3;
          message[3] = b4;
          Serial.write(message, 4);
          b1 =0;

          distance = 0;
          countWheel1 = 0;
          countWheel2 = 0;
          encoderCount1 = 0;  // Reset encoder counts
          encoderCount2 = 0;

        }
        break;
      }
    }

  }
  else
  {
  
    stop();
  }
  delay(50);
}

void CheckMessage()
{
  if (Serial.available())
  {
    Serial.readBytes(data,4);

    received = true;

    if (int(data[0]) == 16)
    {
      // Serial.println("16 received");
      if (int(data[1]) == 1)
      {
        GetSpeed();
        return;

      }
    }

    if (int(data[0])==179)
    {
      b1 =179;
      state =4;
      return;
    }



    if (int(data[0]) == 112)
    {
      b1 = 96;
      state = 1;
      return;
      
    }
    else if (b1 == 96)
    {
      b1 = 97;
    
      return;

    }  

    if(int(data[0]) == 80)
    {
      b1 = 97;
      
      GetSpeed();
      if (int(data[1]) ==1)
      {
        state = 2;
      }

      return;
    }

    if (int(data[0])  == 146)
    {
      if (int(data[2]) == 1)
      {
        state =0 ;
      }
      return;
    }
  
    if(int(data[0]) == 147)
    {
      b1 = 161;
  
      if (int(data[3]) == 0 || int(data[3]) == 1)
      {
        b2 = int(data[2]);
        b3 = int(data[2]);
        GetSpeed();

      }
      else if (int(data[3]) == 2 || int(data[3]) == 3)
      {
        if (int(data[1])==1)
        {
          Angle1 = 255;
        }
        else
        {
          Angle1 = int(data[1]);
        }
        Angle2 = int(data[2]);

      }

      state = 2;
      return;
    }
    
  
    if(int(data[0]) == 145 && int(data[1]) ==1)
    {
      b1 = 228;
      state =3; 
      return;
    }

    if (int(data[0]) == 170)
    {
      received = false;
      return;
    }
    

  }
}

void calibrate()
{ 
  int targetSpeed = Speed;  // Example target speed
  calculateWheelSpeed(targetSpeed);
  return;

}

void calculateWheelSpeed(int targetSpeed) {
  // int currentSpeed = 0;
  // GetSpeed();

  // int sW1 = analogRead(wheel1);
  // int sW2 = analogRead(wheel2);

  // int gaps = 36;
  // int omtrek = 3.14*50; 
  // int counted = 0;

  // int pwm = 0 ;
  // int adcValue = 0;
  // float voltage = 0, prevVolt = 0;
  // const float referenceVoltage = 5.0;  // Default reference voltage for Arduino Nano
  // const int resolution = 1023;

  // unsigned long startMillis = 0, currentMillis = 0;
  // while (currentSpeed < targetSpeed) {
  //   // Get current time
  //   currentMillis = millis();
    
  //   // If interval has passed, update speed
  //   if (currentMillis - startMillis >= 100) {
  //     // Reset timer
  //     startMillis = currentMillis;
      
  //     // Write PWM to motor
  //     analogWrite(pwmPin1, pwm);
  //     analogWrite(dirPin1, 0);
  //     analogWrite(pwmPin2, pwm);
  //     analogWrite(dirPin2, 0);
      
  //     // Read the analog value from pin A0 (photo diode)
  //     adcValue = analogRead(wheel1);
      
  //     // Convert the ADC value to voltage
  //     voltage = (adcValue * referenceVoltage) / resolution;
      
  //     // Detect a rising edge in the voltage (gap detection)
  //     if (voltage > prevVolt) {
  //       counted++;
  //     }
      
  //     // Calculate the current speed based on counted gaps
  //     currentSpeed = (counted * 50 * 3.14) / (32 * 1);  // Adjust this formula as per your wheel's parameters
      
  //     // Increase PWM gradually
  //     pwm += 5;
      
  //     // Save the previous voltage
  //     prevVolt = voltage;

  //     // Ensure PWM does not exceed maximum value (255 for 8-bit PWM)
  //     if (pwm > 255) {
  //       pwm = 255;
  //     }
  //   }
  // }

  // Speed = currentSpeed;
  // savedPWM1 = pwm;
  // savedPWM2 = pwm;
  // GetSpeed();
  int targetPWM = savedPWM1 * 1.02;  // Increase saved PWM by 2%
  int currentPWM = 0;

  // Start the PWM from 0 and gradually increase it
  while (currentPWM < targetPWM) {
    analogWrite(pwmPin1, currentPWM);
    analogWrite(dirPin2, currentPWM);

    delay(1000);  // Delay to allow gradual increase, adjust as needed

    currentPWM += 5;  // Increase PWM gradually by 5 each loop

    // Ensure we don't exceed 255, which is the max PWM value
    if (currentPWM > 255) {
      currentPWM = 255;
    }
  }

  // Stop the motor once we reach the target PWM
  stop();

  float rpm = (currentPWM / 255.0) * 50.0;  // Reverse the RPM-to-PWM conversion

  // Convert RPM back to speed (mm/s) using wheel diameter (50 mm)
  float speed = (rpm * 3.1416 * 50) / 60.0;  // RPM to speed in mm/s

  Speed = speed; 
  

  return;

}

void stop()
{
 
  flag = true;            // Reset the flag to indicate the robot is stopped
  elapsedTimeSeconds = 0;  // Reset elapsed time
  distance = 0;
  countWheel1 = 0;
  countWheel2 = 0;
  encoderCount1 = 0;  // Reset encoder counts
  encoderCount2 = 0;
  startTime = 0;           // Reset the timer
  stopTime = millis(); 
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(dirPin1, 0);
  analogWrite(dirPin2, 0);
  return;
}

void reverse()
{

  if (FF == true) {
    distance = 0;
  countWheel1 = 0;
  countWheel2 = 0;
  encoderCount1 = 0;  // Reset encoder counts
  encoderCount2 = 0 ;// Reset encoder counts
  }
  

  analogWrite(dirPin1, savedPWM1);
  analogWrite(dirPin2, 0);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, savedPWM2);

  FB = true;  // Set reverse flag
  FF = false; // Clear forward flag
  
  return;
}

void forward()
{
  if (FB == true) {
    distance = 0;
  countWheel1 = 0;
  countWheel2 = 0;
  encoderCount1 = 0;  // Reset encoder counts
  encoderCount2 = 0 ; // Reset encoder counts
  }
  analogWrite(pwmPin1, savedPWM1);
  analogWrite(pwmPin2, 0);
  analogWrite(dirPin1, 0);
  analogWrite(dirPin2, savedPWM2);

  FF = true;  // Set forward flag
  FB = false;
  return;
}

void left(int x)
{
  distance = 0;
  countWheel1 = 0;
  countWheel2 = 0;
  encoderCount1 = 0;  // Reset encoder counts
  encoderCount2 = 0;

  float turnTime = x / turnSpeedDegreesPerSecond*2.2; // Time to turn the desired angle

  // Set motor 1 to move backward and motor 2 to move forward
     // Motor 2 forward
  analogWrite(pwmPin1, savedPWM1);
  analogWrite(dirPin1, 0);  // Motor 1 forward

  analogWrite(pwmPin2, savedPWM1);
  analogWrite(dirPin2, 0);  // Motor 2 backward

  delay(turnTime * 1000);  // Convert to milliseconds and wait for the turn to complete

  stop(); 

  distance = 0;
  return;
}

void right(int x)
{

  distance = 0;
  countWheel1 = 0;
  countWheel2 = 0;
  encoderCount1 = 0;  // Reset encoder counts
  encoderCount2 = 0;

  float turnTime = x / turnSpeedDegreesPerSecond*2.2; // Time to turn the desired angle

  analogWrite(dirPin1, savedPWM1);
  analogWrite(pwmPin1, 0);  // Motor 1 backward

  analogWrite(pwmPin2, 0);
  analogWrite(dirPin2, savedPWM1);
  
  delay(turnTime * 1000);  // Convert to milliseconds and wait for the turn to complete

  stop();  // Stop the motors after turning

  distance = 0;
  return;
}

void r360()
{
  analogWrite(pwmPin1, 255);
  analogWrite(dirPin2, 255);
  analogWrite(pwmPin2, 0);
  analogWrite(dirPin1, 0);

  return;
}

void GetSpeed()
{
  //   b2 = int(data[2]);

  if (int(data[2])== 0 && int(data[0]) == 80 )
  {
    Speed = 60;
  }
  else
  {
    Speed = int(data[2]);
  }
  
  
  //   b3= int(data[2]);
  float rpm = (Speed * 60) / (3.1416 * 50);  // Speed in mm/s to RPM conversion

  // Calculate PWM from RPM, assuming max RPM is 50 for full PWM (255)
  int pwm = (rpm / 50) * 255;

  // Ensure PWM is within valid range
  if (pwm > 255) pwm = 255;
  if (pwm < 0) pwm = 0;

  savedPWM1 = pwm;
  savedPWM2 = pwm;
  return;
}

void GetDirection()
{
  if (Angle1 == 255)
  {
    b2 =1;
  }
  else
  {
    b2 = 0;
  }
  b3 = Angle2;

  return;
}



void updateEncoderCounts() {
  unsigned long currentTime = millis();
  
  // Read the current state of the encoders
  int currentEncoderState1 = digitalRead(wheel1);
  int currentEncoderState2 = digitalRead(wheel2);
  
  // Debounce encoder signal for wheel1
  if ((currentTime - lastDebounceTime1) > debounceDelay) {
    if (currentEncoderState1 == HIGH && prevEncoderState1 == LOW) {
      countWheel1++;  // Increase countWheel1 when a gap is detected
      lastDebounceTime1 = currentTime;
    }
  }
  
  // Debounce encoder signal for wheel2
  if ((currentTime - lastDebounceTime2) > debounceDelay) {
    if (currentEncoderState2 == HIGH && prevEncoderState2 == LOW) {
      countWheel2++;  // Increase countWheel2 when a gap is detected
      lastDebounceTime2 = currentTime;
    }
  }

  // Store the current states as previous states for the next loop
  prevEncoderState1 = currentEncoderState1;
  prevEncoderState2 = currentEncoderState2;
  return;
}

void GetDistance()
{
  float circumference = 3.1416 * 50;  // Circumference of the wheel (52 mm diameter)
  int transitionsPerGap =1;  // Assume 10 transitions per gap (based on observations)
  float distancePerGap = circumference / 32;  // Calculate distance per gap

  // Convert transition counts to gap counts
  int actualGapsWheel1 = countWheel1 / transitionsPerGap; 
  int actualGapsWheel2 = countWheel2 / transitionsPerGap;

  // Distance calculation based on actual gap counts
  distance = actualGapsWheel2 * distancePerGap;


  // Prepare values to send over serial
  b2 = 0;  // No overflow handling for now
  b3 = (int)distance;  // Store the distance in byte

  startTime = millis();
  return;
}
