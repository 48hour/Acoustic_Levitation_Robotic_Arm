#include "Wire.h"  //찐_최종(버튼 기능 추가)
#include <MPU6050_light.h>
#include <SoftwareSerial.h>
#include <Servo.h>

MPU6050 mpu(Wire);

Servo servo10;
Servo servo30;

int servoPin1 = 7;
int servoPin3 = 8;


// 이동 평균 필터를 위한 변수들
const int numReadings = 20;
float readingsX[numReadings];
float readingsY[numReadings];
float readingsZ[numReadings];
int index = 0;
float totalX = 0.0;
float totalY = 0.0;
float totalZ = 0.0;

int ReadPin = 6;
int previousReadPin = 0;
int Servo3 = 90;
int previousServo3 = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  servo10.attach(servoPin1);
  servo30.attach(servoPin3);


  byte status = mpu.begin();
  while (status != 0) {} // MPU6050에 연결하지 못하면 모든 것을 멈춥니다.

  delay(200);

  mpu.calcOffsets();

  // 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    readingsX[i] = 0.0;
    readingsY[i] = 0.0;
    readingsZ[i] = 0.0;
  }
}

void loop() {
  
  int currentReadPin = digitalRead(ReadPin);

  mpu.update();

  float currentX = mpu.getAngleX();
  float currentY = mpu.getAngleY();
  float currentZ = mpu.getAngleZ();

  // 이동 평균 필터 적용
  totalX = totalX - readingsX[index] + currentX;
  totalY = totalY - readingsY[index] + currentY;
  totalZ = totalZ - readingsZ[index] + currentZ;
  readingsX[index] = currentX;
  readingsY[index] = currentY;
  readingsZ[index] = currentZ;
  index = (index + 1) % numReadings;

  // 값 변환
  float smoothedX = totalX / numReadings;
  float smoothedY = totalY / numReadings;
  float smoothedZ = totalZ / numReadings;

  int Servo1 = int(-smoothedZ) + 90;
  int Servo3 = int(-smoothedX) + 90;

  Servo1 = constrain(Servo1, 1, 179);
  Servo3 = constrain(Servo3, 75, 140);

    if (currentReadPin != previousReadPin) {
    // ReadPin 값이 변했을 때
    if (currentReadPin == 0) {
      // 1에서 0으로 변하는 순간
      // 이전 Servo3 값을 유지하고 출력

      servo10.write(Servo1);
      servo30.write(previousServo3);
      Serial.println(Servo1);
    }
  }

  previousReadPin = currentReadPin;

  if (currentReadPin == 1) {  
    // 1에서 0으로 변하는 순간부터 Servo3 값을 유지하고 출력
    Servo3 = previousServo3;
      servo10.write(Servo1);
      servo30.write(Servo3);
      Serial.println(Servo1);
  } else {
      servo10.write(Servo1);
      servo30.write(Servo3);
      Serial.println(Servo1);
    
    previousServo3 = Servo3;
  }

  delay(5);
}



/*
#include "Wire.h"  
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

MPU6050 mpu(Wire);

// 이동 평균 필터를 위한 변수들
const int numReadings = 10;
float readingsX[numReadings];
float readingsY[numReadings];
float readingsZ[numReadings];
int index = 0;
float totalX = 0.0;
float totalY = 0.0;
float totalZ = 0.0;

int ReadPin = 6;

int Result = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  //Serial.print(F("MPU6050 status: "));
  //Serial.println(status);
  while(status != 0) {} // MPU6050에 연결하지 못하면 모든 것을 멈춥니다.
  
  delay(200);
  
  //mpu.upsideDownMounting = true; // 만약 MPU6050이 거꾸로 장착되어 있다면 이 줄의 주석을 해제하세요.
  mpu.calcOffsets(); // 자이로스코프와 가속도계의 오프셋 값을 계산합니다.
  //Serial.println("Done!\n");
  
  // 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    readingsX[i] = 0.0;
    readingsY[i] = 0.0;
    readingsZ[i] = 0.0;
  }
}

void loop() {
  mpu.update();
  
  float currentX = mpu.getAngleX();
  float currentY = mpu.getAngleY();
  float currentZ = mpu.getAngleZ();

  // 이동 평균 필터 적용
  totalX = totalX - readingsX[index] + currentX;
  totalY = totalY - readingsY[index] + currentY;
  totalZ = totalZ - readingsZ[index] + currentZ;
  readingsX[index] = currentX;
  readingsY[index] = currentY;
  readingsZ[index] = currentZ;
  index = (index + 1) % numReadings;

  // 값 변환
  float smoothedX = totalX / numReadings;
  float smoothedY = totalY / numReadings;
  float smoothedZ = totalZ / numReadings;

  int Servo3 = int(-smoothedX) + 90;
  int roundedY = int(smoothedY);
  int Servo1 = int(-smoothedZ) + 90;

  
  Result = digitalRead(ReadPin);

  Serial.print(Servo1);
  Serial.print(",");
  Serial.println(Servo3);
  
  delay(100);
}
*/

/*#include "Wire.h"  
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// 이동 평균 필터를 위한 변수들
const int numReadings = 10;
float readingsX[numReadings];
float readingsY[numReadings];
float readingsZ[numReadings];
int index = 0;
float totalX = 0.0;
float totalY = 0.0;
float totalZ = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status != 0) {} // MPU6050에 연결하지 못하면 모든 것을 멈춥니다.
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  //mpu.upsideDownMounting = true; // 만약 MPU6050이 거꾸로 장착되어 있다면 이 줄의 주석을 해제하세요.
  mpu.calcOffsets(); // 자이로스코프와 가속도계의 오프셋 값을 계산합니다.
  Serial.println("Done!\n");
  
  // 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    readingsX[i] = 0.0;
    readingsY[i] = 0.0;
    readingsZ[i] = 0.0;
  }
}

void loop() {
  mpu.update();
  
  float currentX = mpu.getAngleX();
  float currentY = mpu.getAngleY();
  float currentZ = mpu.getAngleZ();

// 이동 평균 필터 적용
  totalX = totalX - readingsX[index] + currentX;
  totalY = totalY - readingsY[index] + currentY;
  totalZ = totalZ - readingsZ[index] + currentZ;
  readingsX[index] = currentX;
  readingsY[index] = currentY;
  readingsZ[index] = currentZ;
  index = (index + 1) % numReadings;

  float smoothedX = totalX / numReadings;
  float smoothedY = totalY / numReadings;
  float smoothedZ = totalZ / ndhumReadings;

  int roundedX = int(smoothedX);
  int roundedY = int(smoothedY);
  int roundedZ = int(smoothedZ);

  //Serial.print("X : ");
  Serial.print(roundedX + 90);
  Serial.print(",");
  //Serial.print("\tY : ");
  //Serial.print(roundedY);
  //Serial.print("\tZ : ");
  Serial.println(roundedZ + 90);

  delay(100);
  
} */
