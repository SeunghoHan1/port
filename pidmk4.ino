#define M1 6
#define E2 4
#define M2 5
#define E1 7

int sensor[4] = { 8,9,10,11 }; // Pin assignment
int sensorReading[4] = { 0 }; // Sensor reading array
int trigPin = 13;
int echoPin = 12;

float activeSensor = 0; // Count active sensors
float totalSensor = 0; // Total sensor readings
float avgSensor = 2.5; // Average sensor reading
float duration = 0;
float distance = 0;
float prev_avg =0;
 
float _Kp =60;// ziegler Nichols, Ku = 120, 0.6 * Ku = 72
float _Ki = 0.20; // Tu = 1, 정확도 상승 높으면 시간이 오래걸림 낮으면 거의변화없음
float _Kd = 22;// 빠르게 목표치까지 접근, 높으면 Overshoot 상승


float error = 0;
float previousError = 0;
float totalError = 0;

float power = 0;

int PWM_Right, PWM_Left;

void setup()
{
  Serial.begin(9600);
  
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  for(int i=0; i<4; i++) {
    pinMode(sensor[i], INPUT);
  }
  
  
 
  
}

void loop()
{
  sonicSensor();
  
 if( distance<=150 && distance >=5) {
   digitalWrite(E1, HIGH);
    digitalWrite(E2, LOW);
     analogWrite(M1, 50);
     analogWrite(M2, 50); 
 }
 
 if(digitalRead(8)==1&&digitalRead(9)==1&&digitalRead(10)==1&&digitalRead(11)==1){
   digitalWrite(E1, HIGH);
    digitalWrite(E2, LOW);
     analogWrite(M1, 80);
     analogWrite(M2, 80); 
 }
 testSensor2();
   lineFollow();
}

void PID_program()
{
  if(digitalRead(8)==0&&digitalRead(9)==0&&digitalRead(10)==0&&digitalRead(11)==0) // IR 센서가 라인을 완전히 벗어난 경우

    {
      if(prev_avg<2.5) // 우코너 진입
        {
          digitalWrite(E1, HIGH); // 회전 내측 모터 right 역회전
          digitalWrite(E2, HIGH); // 회전 외측 모터 left 정회전
          analogWrite(M1, 54); 
          analogWrite(M2, 200); 
        }
      else // 좌코너 진입
        {
          digitalWrite(E1, LOW); // 회전 외측 모터 right 정회전
          digitalWrite(E2, LOW); // 회전 내측 모터 left 역회전
          analogWrite(M1, 200);
          analogWrite(M2, 54); 
        } 
    }
    
  else // 라인 위에 있을 경우 PID 실행 
    {
      readSensor(); 
    
      previousError = error; 
      error = avgSensor - 2.5; // avgSensor 1~4, target value 2.5 
      totalError += error;
      power = (_Kp*error) + (_Kd*(error-previousError)) + (_Ki*totalError); // power 0~255
    
      if( power>255 ) { power = 255.0; }
      if( power<-255 ) { power = -255.0; }
    
      if(power<0)
        {
          PWM_Right = 255;
          PWM_Left = 255 - abs(int(power));
        }
      else
        {
          PWM_Right = 255 - int(power);
          PWM_Left = 255;
        }
      digitalWrite(E1, LOW);
      digitalWrite(E2, HIGH);
      analogWrite(M1, PWM_Left);
      analogWrite(M2, PWM_Right); 
    }
}
   

void lineFollow(void) {

   PID_program();

}

void readSensor(void) 
{
   
  for(int i=0; i<4; i++) 
    {
      sensorReading[i] = digitalRead(sensor[i]);
      if(sensorReading[i]==1) { activeSensor+=1; }
      totalSensor += sensorReading[i] * (i+1);

    }
  avgSensor = totalSensor/(activeSensor+0.0001);
  prev_avg = avgSensor;
  activeSensor = 0; totalSensor = 0;
    

}

void sonicSensor (void) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH,1400);
  distance = ((float)(340 * duration) / 1000) / 2;
}
 




void testSensor2(void) {
   Serial.print(distance);
  Serial.println("");
  delay(20);
}
