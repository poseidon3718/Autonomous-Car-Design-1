#define LINE_DETECT_WHITE 1
#define ledPin 13

#define motor_speed_offset 10

#define ENA 11
#define IN1 28
#define IN2 29
#define IN3 30
#define IN4 31
#define ENB 10

int linesensor_data[5] = {0, 0, 0, 0, 0};
int linesensor_pin[5] = {22, 23, 24, 25, 26};

int read_digital_line_sensor(void){
  
  int i;
  int sum = 0;
  
  for(i=0;i<5;i++){
    if(LINE_DETECT_WHITE == 0){
      linesensor_data[i] = 1 - digitalRead(linesensor_pin[i]);
      }
      else{
        linesensor_data[i] = digitalRead(linesensor_pin[i]);
      }
      sum += linesensor_data[i];
    }
    
  if(sum == 5){
    return sum;
    }
  else if(sum == 2){
    if( (linesensor_data[3] == 1) && (linesensor_data[4] == 1) ) return 3;
    if( (linesensor_data[2] == 1) && (linesensor_data[3] == 1) ) return 1;
    if( (linesensor_data[1] == 1) && (linesensor_data[2] == 1) ) return -1;
    if( (linesensor_data[0] == 1) && (linesensor_data[1] == 1) ) return -3;
    }
  else if(sum == 1){
    if(linesensor_data[0] == 1) return -4;
    if(linesensor_data[1] == 1) return -2;
    if(linesensor_data[2] == 1) return 0;
    if(linesensor_data[3] == 1) return 2;
    if(linesensor_data[4] == 1) return 4;
    }
  else if(sum == 3){
    return -10;
    }
  else{
    return -5;
    }
}



void setup() {
  
  int i;

  pinMode(ledPin, OUTPUT);
  
  for(i=0;i<5;i++){
      pinMode(linesensor_pin[i], INPUT);
    }
  Serial.begin(9600);
}

void motor_A_control(int direction_a, int motor_speed_a) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if(direction_a == HIGH)
  {
     digitalWrite(IN1, HIGH); //모터의 방향 제어
     digitalWrite(IN2, LOW);
     analogWrite(ENA,motor_speed_a); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
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

void line_following(int line_type)
{
  switch(line_type){
    case -5:
          motor_A_control(HIGH,20);
          motor_B_control(HIGH,80);
          break;

    case -4:
      // 왼쪽으로 큰 각도로 회전
          motor_B_control(HIGH,200);
          motor_A_control(LOW,105);
          break;
    case -3:
      // 왼쪽으로 약간 각도로 회전
          motor_B_control(HIGH,180);
          motor_A_control(LOW,65);
          break;

    case -2:
      // 왼쪽으로 약간 각도로 회전
          motor_B_control(HIGH,160);
          motor_A_control(LOW,35);
          break;

    case -1:
      // 왼쪽으로 작은 각도로 회전
          motor_B_control(HIGH,140);
          motor_A_control(HIGH,10);
          break;

    case 0:  
      // 직진
          motor_A_control(HIGH,55);
          motor_B_control(HIGH,80);
          break;

    case 1:
      // 오른쪽으로 작은 각도로 회전
          motor_B_control(HIGH,20);
          motor_A_control(HIGH,115);
          break;

    case 2:
      // 오른쪽으로 약간 각도로 회전
          motor_B_control(LOW,10);
          motor_A_control(HIGH,135);
          break;

    case 3:
      // 오른쪽으로 약간 각도로 회전
          motor_B_control(LOW,40);
          motor_A_control(HIGH,155);
          break;

    case 4:
      // 오른쪽으로 큰 각도로 회전
          motor_B_control(LOW,100);
          motor_A_control(HIGH,175);
          break;
    case 5:
          motor_B_control(LOW,0);
          motor_A_control(HIGH,0);
          break;
    case -10:
      // 선을 발견하지 못한 경우, 정지
          motor_B_control(HIGH,0);
          motor_A_control(LOW,0);
          break;
  }
}

void loop() {
  int i;

  int line_type = 0;
  line_type = read_digital_line_sensor();

  Serial.print("Input data = ");
  
  for(i=0;i<5;i++){
      Serial.print(linesensor_data[i]);
      Serial.print("  ");
    }
    Serial.println(line_type);
    Serial.println("  ");

    line_following(line_type);
}
