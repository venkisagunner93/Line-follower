#include <QTRSensors.h>

#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 9
#define MAX_SPEED 200
#define BASE_SPEED 75
#define Ts 10

int BUZZ = 4;
int EL = 10;
int ER = 11;
int ML = 12;
int MR = 13;

int start_count = 0;
int set_point = (NUM_SENSORS-1)*500;
int error = 0;
int last_error = 0;

float Kp = 0.1;
float Ki = 0;
float Kd = 0;

float proportional_output = 0;
float integral_error = 0;
float integral_output = 0;
float derivative_error = 0;
float derivative_output = 0;

int control_output = 0;
int left_motor_pwm;
int right_motor_pwm;

float bound = 0;

// initialize qtrrc object based on the sensor pins and calibration specifications
QTRSensorsRC qtrrc((unsigned char[]) {0, 1, 2, 3, 5, 6, 7, 8}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 

// sensor values array for the number of sensors used
unsigned int sensorValues[NUM_SENSORS];

void setup() {
// define pin-modes for motors
  pinMode(ML,OUTPUT);
  pinMode(MR,OUTPUT);
  pinMode(EL,OUTPUT);
  pinMode(ER,OUTPUT);

// calibrate the sensors for 10 seconds with each read of 25ms
  delay(500);
  for (int i = 0; i < 400; i++)  
  {
    qtrrc.calibrate();       
  }
// honk once the calibration is complete
  digitalWrite(BUZZ, HIGH);
  delay(500);

//  Serial.begin(9600);
}

void loop() {
  if(start_count < 1){
    delay(5000);   
    start_count = start_count+1; 
  }
  
  unsigned int position = qtrrc.readLine(sensorValues);
  error = set_point - position;

// proportional output
  proportional_output = Kp*error;

// integral output with anti-windup
  if(integral_error > bound){
    integral_error = bound;
  }
  else if(integral_error < -bound){
    integral_error = -bound;  
  }
  else{
    integral_error = integral_error + error;  
  }
  integral_output = Ki*integral_error;

// derivative output
  derivative_error = error - last_error;
  last_error = error;
  derivative_output = Kd * derivative_error;

  control_output = proportional_output + integral_output + derivative_output;

  left_motor_pwm = BASE_SPEED + control_output;
  right_motor_pwm = BASE_SPEED - control_output;

  if(left_motor_pwm >= MAX_SPEED){
    digitalWrite(ML,HIGH);
    left_motor_pwm = MAX_SPEED;  
  }
  else if(left_motor_pwm <= -MAX_SPEED){
    digitalWrite(ML,LOW);
    left_motor_pwm = MAX_SPEED;
  }
  else if(left_motor_pwm <= 0 && left_motor_pwm >= -MAX_SPEED){
    digitalWrite(ML,LOW);
    left_motor_pwm = -left_motor_pwm;
  }
  else{
    digitalWrite(ML,HIGH);  
  }

  if(right_motor_pwm >= MAX_SPEED){
    digitalWrite(MR,HIGH);
    right_motor_pwm = MAX_SPEED;  
  }
  else if(right_motor_pwm <= -MAX_SPEED){
    digitalWrite(MR,LOW);
    right_motor_pwm = MAX_SPEED;
  }
  else if(right_motor_pwm <= 0 && right_motor_pwm >= -MAX_SPEED){
    digitalWrite(MR,LOW);
    right_motor_pwm = -right_motor_pwm;
  }
  else{
    digitalWrite(MR,HIGH);  
  }

  analogWrite(EL,left_motor_pwm);
  analogWrite(ER,right_motor_pwm);
  delay(Ts);
}
