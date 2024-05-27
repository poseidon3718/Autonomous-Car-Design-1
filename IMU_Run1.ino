#include <Wire.h>
#include <LSM303.h>

#define THRESHOLD_ANGLE 10

#define ENA 10
#define IN1 28
#define IN2 29
#define IN3 30
#define IN4 31
#define ENB 11

LSM303 compass;

float heading_angle = 0.0;
float in_heading_angle = 17.0;
float target_heading_angle = 90;
float heading_angle_error = 0.0;

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}

void motor_A_control(int direction_a, int motor_speed_a) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if(direction_a == HIGH)
  {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
     analogWrite(ENA,motor_speed_a); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN1, LOW); //모터의 방향 제어
    digitalWrite(IN2, HIGH);
    analogWrite(ENA,motor_speed_a);
    
  }
}

void motor_B_control(int direction_b, int motor_speed_b) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if(direction_b == HIGH)
  {

     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
     analogWrite(ENB,motor_speed_b); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN3, HIGH); //모터의 방향 제어
    digitalWrite(IN4, LOW);
    analogWrite(ENB,motor_speed_b);
    
  }
}


void imu_rotation(void)
{
  compass.read();
  float heading1 = compass.heading();
  compass.read();
  float heading2 = compass.heading();
  float heading = (heading1 + heading2)/2;

  heading = 360 - heading;

  Serial.println(heading);

  heading_angle_error = target_heading_angle - heading;

  if(heading_angle_error > 180)
  {
    heading_angle_error = heading_angle_error - 360;
  }
  else if(heading_angle_error < -180)
  {
    heading_angle_error = heading_angle_error + 360;
  }

  Serial.print("Heading Angle Error : ");
  Serial.print(heading_angle_error);
  Serial.print(" = ");
  Serial.print(target_heading_angle);
  Serial.print(" - ");
  Serial.println(heading);

  if(heading_angle_error > THRESHOLD_ANGLE)
  {
    motor_A_control(HIGH,50);
    motor_B_control(LOW,50);
  }
  else if((heading_angle_error >= -THRESHOLD_ANGLE)&&(heading_angle_error <= THRESHOLD_ANGLE))
  {
    motor_A_control(HIGH,0);
    motor_B_control(HIGH,0);
  }
  else
  {
    motor_A_control(LOW,50);
    motor_B_control(HIGH,50);
  }
}

void loop() {
  imu_rotation();
}