#include <Wire.h>
#include <LSM303.h>
#include <NewPing.h>

bool flag[6] = {1,1,1,1,1,1};

LSM303 compass;

float heading = 0.0;
float target_heading_angle = 0.0;

#define SONAR_NUM 3                // 초음파 센서 개수 = 3
#define MAX_DISTANCE 150           // 최대 거리 cm 단위
#define WALL_GAP_DISTANCE      400 //mm 단위
#define WALL_GAP_DISTANCE_HALF 150 //mm 단위
#define MOTOR_PWM_OFFSET 10        // PWM offset 정의

#define Front 0  // 앞쪽 초음파 센서 번호 0
#define Left  1  // 왼쪽 초음파 센서 번호 1
#define Right 2  // 오른쪽 초음파 센서 번호 2

#define TRIG1 14 // 초음파 센서 1번 Trig 핀 14
#define ECHO1 15 // 초음파 센서 1번 Echo 핀 15

#define TRIG2 17 // 초음파 센서 2번 Trig 핀 17
#define ECHO2 16 // 초음파 센서 2번 Echo 핀 16

#define TRIG3 19 // 초음파 센서 3번 Trig 핀 19
#define ECHO3 18 // 초음파 센서 3번 Echo 핀 18

NewPing sonar[SONAR_NUM] = {  // 초음파 센서 개수에 맞는 값 지정
NewPing(TRIG1, ECHO1, MAX_DISTANCE), // 각 초음파 센서 최대거리 설정
NewPing(TRIG2, ECHO2, MAX_DISTANCE),
NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};

/////////////////////L298//////////////////////////

#define IN1 29 // 왼쪽 모터 제어 핀 29
#define IN2 28 // 왼쪽 모터 제어 핀 28
#define ENA 10 // 왼쪽 속 제어 핀 10

#define IN3 30 // 오른쪽 모터 제어 핀 30
#define IN4 31 // 오른쪽 모터 제어 핀 31
#define ENB 11 // 오른쪽 속도 제어 핀 11

 float front_sonar = 0.0; // 각 초음파 센서 초기값 설정
 float left_sonar  = 0.0;
 float right_sonar = 0.0;

/////////////////////Maze_Status//////////////////////////

int maze_status = 0; // 미로 탈출 초기값 설정

void setup() 
{
  pinMode(TRIG1, OUTPUT); // 각 초음파 센서 입출력 설정
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  
  pinMode(IN1, OUTPUT); // 모터 제어 출력 설정
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Wire.begin();
  compass.init();
  compass.enableDefault();
  flag[0] = 0;  
  
  Serial.begin(9600); // 통신속도를 9600으로 정의 
}

void motor_A_control(int direction_a, int motor_speed_a) // 모터 A(왼쪽)의 방향(direction)과 속도(speed) 제어
{
  if(direction_a == HIGH)
  {
     digitalWrite(IN1, LOW); // LOW, HIGH 신호로 모터 방향 제어
     digitalWrite(IN2, HIGH);
     analogWrite(ENA,motor_speed_a); // 모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA,motor_speed_a);
  }
}

void motor_B_control(int direction_b, int motor_speed_b) // 모터 B(오른쪽)의 방향(direction)과 속도(speed) 제어
{
  if(direction_b == HIGH)
  {
     digitalWrite(IN3, LOW); // 모터의 방향 제어
     digitalWrite(IN4, HIGH);
     analogWrite(ENB,motor_speed_b); // 모터의 속도 제어
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB,motor_speed_b);
  }
}

//////////////////////imu///////////////////////

void imu_rotation_right(void)
{
  if(target_heading_angle>=360)
  {
    while(1)
    {
      compass.read();
      heading = compass.heading();
      if(heading<180)
      {
          heading+=360;
      }        
      motor_A_control(HIGH,250);
      motor_B_control(LOW,250);
    
      if(heading >= target_heading_angle)
      {
          break;
      }
    }
  }
  else
  {
    while(heading < target_heading_angle)
    {
      compass.read();
      heading = compass.heading();
    
      motor_A_control(HIGH,250);
      motor_B_control(LOW,250);
    
      if(heading >= target_heading_angle)
      {
          break;
      }
    }
  }
}

void imu_rotation_left(void)
{
  if(target_heading_angle<=0)
  {
    while(1)
    {
      compass.read();
      heading = compass.heading();
      if(heading>180)
      {
          heading-=360;
      }    
    
      motor_A_control(LOW,250);
      motor_B_control(HIGH,250);
    
      if(heading <= target_heading_angle)
      {
          break;
      }
    }
  }
  else
  {
    while(heading > target_heading_angle)
    {
      compass.read();
      heading = compass.heading();
      heading += 360;
    
      motor_A_control(LOW,250);
      motor_B_control(HIGH,250);
    
      if(heading <= target_heading_angle)
      {
          break;
      }
    }
  }
}

void wall_collision_avoid(int base_speed)
{
  float error = 0.0;
  float Kp = 0.3;    // 얼마나 회전할 지 설정
  int pwm_control = 0;
  int right_pwm = 0;
  int left_pwm  = 0;
  error = (right_sonar - left_sonar); // 오른쪽 초음파 측정값과 왼쪽 초음파 측정값의 차를 error 값으로 지정
  error = Kp * error; // error 값에 kp를 곱하여 오차값을 소폭 증가시킴 (회전값 증가)
  
  if(error >= 50) error = 50; // error값이 50이상일 때 50으로 조정
  if(error <= -50) error = -50; // error값이 -50이하일 때 -50으로 조정
                      
  right_pwm = base_speed - error; 
  left_pwm  = base_speed + error;
  
  if(right_pwm <= 0) right_pwm = 0;
  if(left_pwm  <= 0) left_pwm  = 0;

  if(right_pwm >= 255) right_pwm = 255;
  if(left_pwm  >= 200) left_pwm  = 200;
  
  motor_A_control(HIGH,left_pwm);  // 오른쪽 전진
  motor_B_control(HIGH,right_pwm); // 왼쪽 전진
}

void loop() 
{
  front_sonar = sonar[Front].ping_cm()*8; // 전방 센서 측정
  left_sonar  = sonar[Left].ping_cm() *8; // 좌축 센서 측정
  right_sonar = sonar[Right].ping_cm()*8; // 우측 센서 측정
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE*10; // 0.0 출력이 최대값이므로
  if(left_sonar  == 0.0)  left_sonar = MAX_DISTANCE*10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE*10;


   if(flag[0]==0)
  {
    wall_collision_avoid(215);
    
    if(left_sonar>=WALL_GAP_DISTANCE)
    {
      motor_A_control(HIGH,150); 
      motor_B_control(HIGH,255); 
      delay(270); // 첫 코너 직진           
         
      compass.read();
      heading = compass.heading();
      
      target_heading_angle = heading - 90;
      
      imu_rotation_left();
      
      flag[0] = 1;
      flag[1] = 0;
    }
  }
  else if(flag[1]==0)
  {
    wall_collision_avoid(215);
    
    if(front_sonar<=WALL_GAP_DISTANCE_HALF)
    {
      compass.read();
      heading = compass.heading();
      
      target_heading_angle = heading + 90;  
           
      imu_rotation_right(); // 우회전
      
      flag[1] = 1;
      flag[2] = 0;
    }
  }
  else if(flag[2]==0)
  {
    wall_collision_avoid(215);
   if(front_sonar<=WALL_GAP_DISTANCE_HALF)
    {
      compass.read();
      heading = compass.heading();
      
      target_heading_angle = heading + 180;    
      
      imu_rotation_right(); // 유턴
      
      flag[2] = 1;
      flag[3] = 0;
    }
  }
  else if(flag[3]==0)
  {
    wall_collision_avoid(215);
    if(front_sonar<=WALL_GAP_DISTANCE_HALF)
    {
      compass.read();
      heading = compass.heading();
      
      target_heading_angle = heading - 90;
      
      imu_rotation_left(); // 좌회전
      
      flag[3] = 1;
      flag[4] = 0;
    }
  }
  else if(flag[4]==0)
  {
    wall_collision_avoid(215);
    if(front_sonar<=WALL_GAP_DISTANCE_HALF)
    {
      compass.read();
      heading = compass.heading();
      
      target_heading_angle = heading - 90;
      
      imu_rotation_left(); // 좌회전
      
      flag[4] = 1;
      flag[5] = 0;
    }
  }
  else if(flag[5]==0)
  {
    wall_collision_avoid(215);
  }
 }