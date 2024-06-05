/////////////////////sonar///////////////////////

#include <NewPing.h>

#define SONAR_NUM 3      // 소나 센서 개수
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE      350//mm 단위
#define WALL_GAP_DISTANCE_HALF 200 //mm 단위
#define MOTOR_PWM_OFFSET 10   //모터간 속도 차이 10


#define Front 0
#define Left  1 
#define Right 2

#define TRIG1 2 // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 3 // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 6 // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 7 // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 4 // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 5 // 초음파 센서 3번 Echo 핀 번호

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing(TRIG2, ECHO2, MAX_DISTANCE),
NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};


/////////////////////L298//////////////////////////
#define ENA 12
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
#define ENB 13

 float front_sonar = 0.0;
 float left_sonar  = 0.0;
 float right_sonar = 0.0;

/////////////////////Maze_Status//////////////////////////
int maze_status = 0;    //maze 기본상태 0


void setup() 
{

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
   Serial.begin(115200); // 통신속도를 115200으로 정의함
  
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
     digitalWrite(IN3, HIGH); //모터의 방향 제어
     digitalWrite(IN4, LOW);
     analogWrite(ENB,motor_speed_b); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB,motor_speed_b);
    
  }
}



void check_maze_status(void)
{
  if((left_sonar>=0) && (left_sonar<=WALL_GAP_DISTANCE) && (right_sonar>=0) && (right_sonar<=WALL_GAP_DISTANCE) && (front_sonar>=0) && (front_sonar<=WALL_GAP_DISTANCE_HALF)) // 3면이 다 막힌 경우
  {
    maze_status = 4;
    Serial.println("maze_status = 4");
  }
  else if( (left_sonar>=0) && (left_sonar<=WALL_GAP_DISTANCE) && (right_sonar>=0) && (right_sonar<=WALL_GAP_DISTANCE) && (front_sonar>=WALL_GAP_DISTANCE_HALF)  )     // 좌우 면이 막힌 경우
  {
    maze_status = 1;
    Serial.println("maze_status = 1");
  }
  else if((left_sonar>=0) && (left_sonar<=WALL_GAP_DISTANCE) && (front_sonar>=0) && (front_sonar<=WALL_GAP_DISTANCE_HALF))    //왼쪽과 앞면이 막힌 경우
  {
    maze_status = 2;
    Serial.println("maze_status = 2");
  }
  else if((right_sonar>=0) && (right_sonar<=WALL_GAP_DISTANCE) && (front_sonar>=0) && (front_sonar<=WALL_GAP_DISTANCE_HALF))    // 오른쪽과 앞면이 막힌 경우
  {
    maze_status = 3;
    Serial.println("maze_status = 3");
  }
  else      // 지금까지의 경우들을 제외한 나머지의 경우
  {
    maze_status = 0;
    Serial.println("maze_status = 0"); 
  }
}
//먼저 left_sonar = 0; right_sonar = 0; 으로 해서 오른쪽 방향 찾기
void wall_collision_avoid(int base_spped)
{
  float error = 0.0;
  float Kp = 1.2; //나중에 조정해야 할 값(얼마나 돌지)
  int pwm_control = 0;
  int right_pwm = 0;
  int left_pwm  = 0;
  error = (right_sonar - left_sonar);   //오른쪽 소나 측정값과 왼쪽소나 측정값을 뺀 값을 error 변수에 지정해준다.
  error = Kp * error;     // error 값에 kp를 곱하여 오차값을 소폭 증가시켜준다.(회전값 증가)

  //모터 최고속도가 255이므로 255를 초과하지 않기위해 error값 조정
  if(error >= 50) error = 50;     //error값이 50이상이라면 50으로 조정
  if(error <= -50) error = -50;     //error값이 -50이하라면 -50으로 조정
                       

  right_pwm = base_spped - error;     // right_pwm 변수에 기본 전진속도 오른쪽모터에서 error 를 빼준다.
  left_pwm  = base_spped + error;     // left_pwm 변수에 기본 전진 왼쪽 모터속도에서 error를 더해준다.
  
  if(right_pwm <= 0) right_pwm = 0;   //right_pwm 값이 0이하가 되어버렸다면 0으로 맞춘다.
  if(left_pwm  <= 0) left_pwm  = 0;   //left_pwm 값이 0이하가 되어버렸다면 0으로 맞춘다.ㅏ

  if(right_pwm >= 247) right_pwm = 247;   //right_pwm 값이 직진 최고속도값인 247이상이 되어버렸다면 최댓값으로 맞춘다.
  if(left_pwm  >= 250) left_pwm  = 250;   //left_pwm 값이 직진 최고속도값인 250이상이 되어버렸다면 최댓값으로 맞춘다.
  
  motor_A_control(HIGH,left_pwm); //오른쪽 전진
  motor_B_control(HIGH,right_pwm); // 왼쪽 전진
  
}
void loop() 
{
  //Serial.print("Duration: ");
  //Serial.println(duration);
 
  front_sonar = sonar[Front].ping_cm()*10; // 전방 센서 측정
  left_sonar  = sonar[Left].ping_cm() *10; // 좌축 센서 측정
  right_sonar = sonar[Right].ping_cm()*10; // 우측 센서 측정
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE*10; // 0.0 출력이 최대값이므로
  if(left_sonar  == 0.0)  left_sonar = MAX_DISTANCE*10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE*10;

  Serial.print("L: "); Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F: "); Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R: "); Serial.println(right_sonar);

  check_maze_status();

  if(maze_status == 4)
  {
  //정지 한다
  Serial.println("Rotate CCW");
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  delay(50);
  
  //180도 회전을 한다
  motor_A_control(HIGH,220); //왼쪽 전진
  motor_B_control(LOW,230); //오른쪽은 후진
  delay(665);
  
  //정지 한다
  Serial.println("Rotate CCW");
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  delay(50);
  
  //앞으로 간다
  Serial.println("Go Straight");
  motor_A_control(HIGH,100);
  motor_B_control(HIGH,110);

  }
  
  if(maze_status == 1)
  {
  //앞으로 간다
  Serial.println("run straight");
  wall_collision_avoid(215);
  } 
  
  else if(maze_status == 3)
  {
  //정지 한다
  Serial.println("Rotate CCW");
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  delay(100);
  
  //왼쪽으로 90도 회전 한다
  motor_A_control(LOW,220); 
  motor_B_control(HIGH,230); 
  delay(305);
  //정지 한다 
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  delay(100);
  }
  
  else if(maze_status == 2)
  {
  //정지 한다
  Serial.println("Rotate CCW");
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  delay(100);
  
  //오른쪽으로 90도 회전 한다
  motor_A_control(HIGH,220); 
  motor_B_control(LOW,230); 
  delay(315);
  //정지 한다
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  delay(100);
  } 
  
  else
  {
   //앞으로 간다
  Serial.println("Go Straight");
  motor_A_control(HIGH,240);
  motor_B_control(HIGH,250);
  }

 }
