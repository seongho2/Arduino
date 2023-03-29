#include <NewPing.h>

#define SONAR_NUM 1 // 초음파 센서 1개    
#define MAX_DISTANCE 150 // 최대 인식 거리

#define Front 0

#define TRIG 2  //  초음파 센서 Trig 핀 
#define ECHO 3  //  초음파 센서 Echo 핀

NewPing sonar(TRIG, ECHO, MAX_DISTANCE);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); 
  Serial.begin(115200); // 통신 속도를 115200으로 설정
}

long sonar_front(void) {
  long duration, distance;

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  distance = ((float)(340 * duration) / 1000) / 2;
  
  return distance;
}

void loop() {
  float front_sonar = 0.0;  // 거리를 초기화

  front_sonar = sonar.ping_cm()*10;  //  단위를 cm에서 mm으로 변경
  if(front_sonar == 0.0)  front_sonar = MAX_DISTANCE;
   
  Serial.print("Distance: ");
  Serial.print(front_sonar);  // 거리를 출력
  Serial.println("mm");

  if((front_sonar > 0) && (front_sonar <= 250.0)){  // 250mm보다 가까우면 LED 켜짐, 멀어지면 LED 꺼짐
    digitalWrite(LED_BUILTIN, HIGH);   // LED 켜짐
    delay(500);  // 0.5초동안 켜짐
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);    // LED 꺼짐
    delay(500);  // 0.5초동안 꺼짐
  }
}
