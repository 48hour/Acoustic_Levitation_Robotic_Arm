#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

int servoPin1 = 4; 
int servoPin2 = 6;
int servoPin3 = 8;
int servoPin4 = 10;
int servoPin5 = 12;
int controlPin = 37;
int redLED = 50;
int blueLED = 22;
int power = 36;
int Lev = 7;


int angle1 = 90;
int angle2 = 90;
int angle3 = 90;
int angle4 = 90;
int angle5 = 90;

void setup() {
  Serial.begin(115200);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
  servo4.write(angle4);
  servo5.write(angle5);

  pinMode(controlPin, OUTPUT);

  pinMode(Lev, INPUT);

  digitalWrite(controlPin, HIGH); // 초기값: 스위치 OFF
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, LOW);
  pinMode(blueLED, OUTPUT);
  digitalWrite(blueLED, LOW);
  pinMode(power, OUTPUT);
  digitalWrite(power, LOW);
}

void loop() {
  long value1 = 0;
  long value2 = 0;
  long value3 = 0;
  long value4 = 0;
  long value5 = 0;
  int controlValue = 0; // 6번째 입력값을 저장할 변수
  
  if (Serial.available() >= 6) {
    value1 = Serial.parseInt();
    value2 = Serial.parseInt();
    value3 = Serial.parseInt();
    value4 = Serial.parseInt();
    value5 = Serial.parseInt();
    controlValue = Serial.parseInt(); // 6번째 입력값 읽기
  }
  
  if (value1 > 0 && value1 <= 180 && value2 > 0 && value2 <= 180 && value3 > 0 && value3 <= 180 && value4 > 0 && value4 <= 180 && value5 > 0 && value5 <= 180) {
    int delta1 = abs(angle1 - value1);
    int delta2 = abs(angle2 - value2);
    int delta3 = abs(angle3 - value3);
    int delta4 = abs(angle4 - value4);
    int delta5 = abs(angle5 - value5);
    
    int maxDelta = max(delta1, max(delta2, max(delta3, max(delta4, delta5))));
    for (int i = 0; i < maxDelta; i++) {
      if (i < delta1) {
        if (value1 > angle1) {
          angle1++;
        } else {
          angle1--;
        }
        servo1.write(angle1);
      }
      if (i < delta2) {
        if (value2 > angle2) {
          angle2++;
        } else {
          angle2--;
        }
        servo2.write(angle2);
      }
      if (i < delta3) {
        if (value3 > angle3) {
          angle3++;
        } else {
          angle3--;
        }
        servo3.write(angle3);
      }
      if (i < delta4) {
        if (value4 > angle4) {
          angle4++;
        } else {
          angle4--;
        }
        servo4.write(angle4);
      }
      if (i < delta5) {
        if (value5 > angle5) {
          angle5++;
        } else {
          angle5--;
        }
        servo5.write(angle5);
      }
      delay(30);
    }
    

  }

  int A = digitalRead(Lev);

      if (A == 0) {
      digitalWrite(power, LOW);
      digitalWrite(controlPin, HIGH);  // 초음파 꺼짐
      digitalWrite(redLED, HIGH); 
      digitalWrite(blueLED, LOW);  
    
    } else if (A == 1) {
      digitalWrite(power, HIGH);
      digitalWrite(controlPin, LOW);  // 초음파 켜짐
      digitalWrite(redLED, LOW);   
      digitalWrite(blueLED, HIGH); 
}
}
