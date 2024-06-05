#include <Wire.h>
#include <LSM303.h>
#include <NewPing.h>

#define LINE_DETECT_WHITE  1

#define motor_speed_offset 0

LSM303 compass;

#define IN1 29
#define IN2 28
#define ENR 10

#define IN3 30
#define IN4 31
#define ENL 11

#define MAX_DISTANCE 150

#define TRIG1 14 // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 15 // 초음파 센서 1번 Echo 핀 번호

float front_sonar = 0.0;

int linesensor_data[5] = {0,0,0,0,0};  // 읽은 값을 저장할 변수
int linesensor_pin[5] = {22,23,24,25,26};   // int형 배열
int sum;

NewPing sonar_Front = NewPing(TRIG1, ECHO1, MAX_DISTANCE);

void setup()
{
  
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  // put your setup code here, to run once:
  Serial.begin(9600);
  int i;
  for(i=0;i<5;i++)
  {
    pinMode(linesensor_pin[i],INPUT);
  }

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}

void motor_l(int speed)
{
    if (speed >= 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENL, speed); // 0-255
    }
    else
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENL, -speed);
    }
}

void motor_r(int speed)
{
    if (speed >= 0)
    {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENR, speed); // 0-255
    }
    else
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENR, -speed);
    }
}

void robot_control(int left_motor_speed, int right_motor_speed)
{
    motor_l(left_motor_speed);
    motor_r(right_motor_speed);
}

int read_digital_line_sensor(void)
{
  int i;
  sum=0;
  for(i=0;i<5;i++)
  {
    if(LINE_DETECT_WHITE == 0)
    {
      linesensor_data[i] = 1 - digitalRead(linesensor_pin[i]);
    }
    else
    {
      linesensor_data[i] = digitalRead(linesensor_pin[i]);
    }
    sum += linesensor_data[i];
  }

  if(sum == 5)
  {
    return sum;
  }
  else if(sum == 2)
  {
    if( (linesensor_data[3] == 1) && (linesensor_data[4] == 1) ) return -3;
    if( (linesensor_data[2] == 1) && (linesensor_data[3] == 1) ) return -1;
    if( (linesensor_data[1] == 1) && (linesensor_data[2] == 1) ) return 1;
    if( (linesensor_data[0] == 1) && (linesensor_data[1] == 1) ) return 3;
  }
  else if(sum == 1)
  {
    if((linesensor_data[0] == 1)) return 4;
    if((linesensor_data[1] == 1)) return 2;
    if((linesensor_data[2] == 1)) return 0;
    if((linesensor_data[3] == 1)) return -2;
    if((linesensor_data[4] == 1)) return -4;
  }
  else if(sum == 3)
  {
    return -10;
  }
  else
  {
    return -5;
  }
}

void line_following(int line_type)

{
  switch(line_type)
  {
     case -5:
              robot_control(50+motor_speed_offset,50);  
              break;
     case -4:
              robot_control(-80, 200);//왼  
              break;
     case -3:
              robot_control(-40+motor_speed_offset, 180);  
              break;
     case -2:
              robot_control(50+motor_speed_offset, 120);  
              break;                            
     case -1:
              robot_control(10+motor_speed_offset, 70);  
              break;
     case 0:
              robot_control(60+motor_speed_offset, 65);//직
              break;
     case 1:
              robot_control(70+motor_speed_offset, 10);
              break;
     case 2:
              robot_control(120+motor_speed_offset, 50);
              break;
     case 3:
              robot_control(180+motor_speed_offset, -40);
              break;
     case 4:
              robot_control(200+motor_speed_offset, -80);
              break;
     case 5: //불이 다 들어올 경우 정지
              robot_control(0,0);
              break;
   }
 }


void loop()
{
  front_sonar = sonar_Front.ping_cm()*10;
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE*10;

  compass.read();
  float heading = compass.heading();
  float target_angle;

  
  int i;
  int line_type = 0;
  line_type = read_digital_line_sensor();
   
  Serial.print("Input data = ");
  Serial.println(front_sonar);
  Serial.print(heading);

  if(front_sonar<300)
    {
      target_angle = heading-30;
      while(heading>target_angle)
      {
        robot_control(5,150);
      }
      robot_control(50, 50);
      delay(1000);
      target_angle+=60;
      while(heading<target_angle)
      {
        robot_control(150,5);
      }
      robot_control(50+motor_speed_offset, 50);
      delay(500);
      target_angle-=30;
      while(heading>target_angle)
      {
        robot_control(5,150);
      }
    }
  else
  {
    for(i=0;i<5;i++)
    {
      line_following(line_type);
    }
  }
}
