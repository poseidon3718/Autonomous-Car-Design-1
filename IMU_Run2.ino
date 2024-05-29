// IMU-Run1 

#include <Wire.h>
#include <LSM303.h>

#define THRESHOLD_ANGLE 10
#define THRESHOLD_ANGLE1 15
#define THRESHOLD_ANGLE2 7

#define NO_MISSION 4 // 추후에 수정 8개 정도

bool mission_flag[NO_MISSION]={0,0,0,0};
int flag_runner = -1;

LSM303 compass;

float heading_angle = 0.0;
float init_heading_angle = 17.0;
float target_heading_angle = 90;
float heading_angle_error = 0.0;

#define IN1 29
#define IN2 28
#define ENA 10
#define IN3 30
#define IN4 31
#define ENB 11

void setup() 
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(115200); // 통신속도를 115200으로 정의함

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
     digitalWrite(IN1, LOW); //모터의 방향 제어
     digitalWrite(IN2, HIGH);
     analogWrite(ENA,motor_speed_a); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA,motor_speed_a);
  }
}

void motor_B_control(int direction_b, int motor_speed_b) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if(direction_b == HIGH)
  {
     digitalWrite(IN3, LOW); //모터의 방향 제어
     digitalWrite(IN4, HIGH);
     analogWrite(ENB,motor_speed_b); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB,motor_speed_b);
  }
}

int read_imu_sensor(void){
  compass.read();
  float heading1 = compass.heading();
  Serial.println(heading1);
  compass.read();
  float heading2 = compass.heading();
  Serial.println(heading2);
  float heading = (heading1+heading2)/2;

  //heading = 360 - heading;
  
  Serial.println(heading);

  heading_angle_error = target_heading_angle - heading;

  if(heading_angle_error > 180)
  {
    heading_angle_error = heading_angle_error - 360;
  }
  else if(heading_angle_error < 180)
  {
    heading_angle_error = heading_angle_error + 360;
  }
  else{}
  
  Serial.print("Heading Angle Error: ");
  Serial.print(heading_angle_error);
  Serial.print(" = ");
  Serial.print(target_heading_angle);
  Serial.print(" - ");
  Serial.println(heading);

}

void imu_rotation(void)
{
  bool flag=1;
  while(flag)
  {
    read_imu_sensor();

    if(heading_angle_error > THRESHOLD_ANGLE1)
    {
        motor_A_control(LOW,50); 
        motor_B_control(HIGH,50);
    }
    else if((heading_angle_error >= THRESHOLD_ANGLE2)&&(heading_angle_error <= THRESHOLD_ANGLE1))
    {
        motor_A_control(LOW,40); 
        motor_B_control(HIGH,40);
    }
    else if((heading_angle_error > -THRESHOLD_ANGLE2)&&(heading_angle_error < THRESHOLD_ANGLE2))
    {
        motor_A_control(HIGH,0); 
        motor_B_control(HIGH,0);
        flag = 0;
    }
    else if((heading_angle_error >= -THRESHOLD_ANGLE1)&&(heading_angle_error <= -THRESHOLD_ANGLE2))
    {
        motor_A_control(HIGH,35); 
        motor_B_control(LOW,35);
    }
    else
    {
        motor_A_control(HIGH,35); 
        motor_B_control(LOW,35);
    }
  }

  /*
  if(heading_angle_error > THRESHOLD_ANGLE)
  {
    motor_A_control(HIGH,50); motor_B_control(LOW,50);
  }
  else if((heading_angle_error >= -THRESHOLD_ANGLE)&&(heading_angle_error <= THRESHOLD_ANGLE))
  {
    motor_A_control(HIGH,0); motor_B_control(LOW,0);
  }
  else
  {
    motor_A_control(LOW,50); motor_B_control(HIGH,50);
  }
  */
}

void loop(){
  target_heading_angle = 90;
  imu_rotation();

  /*
  // line sensor, sonar sensor, wall following mode 를 flag를 이용하여 모두 사용할 때
  int line_index = -10;

  line_index = read_line_sensor(); // read_line_sensor() 함수 지정 필요

  mission_flag[0] = 0;

  if(mission_flag[0]==0)
  {
    read_sonar_sensor(); // read_sonar_sensor() 함수 지정 필요
    if(front_sonar < 200) // 장애물 감지 후 통과
    {
      mission_flag[0] = 1; 
      mission_flag[1] = 0;
    }
    else if((line_index == 0)&&(right_sonar <= 100)&&(left_sonar <= 100))
    {
      mission_flag[2] = 0;
    }
    else
    {
      line_following(line_index);
    }
  }
  else if(mission_flag[1] == 0)
  {
    obstacle_avoidance(); // obstacle_avoidance() 함수 지정 필요
    mission_flag[1] = 1;
  }
  else if (mission_flag[2] == 0)
  {
    target_angle = 90;
    imu_rotation();
    mission_flag[2] = 1;
    mission_flag[3] = 0;
    if(front_sensor < 10)  // 10cm 이하에 오면

    {

    robot_control(0,0); // 직진

    mission_flag[3] = 1;

    mission_flag[4] = 0;

    }
  }
  */
}
