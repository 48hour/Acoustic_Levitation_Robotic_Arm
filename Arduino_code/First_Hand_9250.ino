#include <MPU9250_WE.h>  //찐_최종
#include <Wire.h>
#include <Servo.h>

#define MPU9250_ADDR_0 0x68 //GND 핀 설정
#define MPU9250_ADDR_1 0x69 //3V3 핀 설정

Servo servo20;
Servo servo40;
Servo servo50;

int servoPin2 = 6;
int servoPin4 = 7;
int servoPin5 = 8;


MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR_0);
MPU9250_WE myMPU9250_1 = MPU9250_WE(MPU9250_ADDR_1);

float gravityX = 0, gravityY = 0, gravityZ = 0;
float pureAccX = 0.0, pureAccY = 0.0, pureAccZ = 0.0;

float gravityX_1 = 0, gravityY_1 = 0, gravityZ_1 = 0;
float pureAccX_1 = 0.0, pureAccY_1 = 0.0, pureAccZ_1 = 0.0;

int ReadPin1 = 11;
int ReadPin2 = 12;

int previousReadPin1 = 0;
int Servo4 = 90;
int Servo5 = 90;
int previousServo4 = 90;
int previousServo5 = 90;

int Result1 = 0;
int Result2 = 0;

const int numReadings = 20; // 이동평균 필터의 샘플 수
float angleXReadings[numReadings]; // 롤(Roll) 값을 저장할 배열
float angleXReadings_1[numReadings]; // 롤(Roll) 값을 저장할 배열
float angleYReadings[numReadings]; // 피치(Pitch) 값을 저장할 배열
float angleYReadings_1[numReadings]; // 피치(Pitch) 값을 저장할 배열
float angleZReadings[numReadings]; // 요(Yaw) 값을 저장할 배열
float angleZReadings_1[numReadings]; // 요(Yaw) 값을 저장할 배열
int angleXIndex = 0;
int angleYIndex = 0;
int angleZIndex = 0;
float angleXTotal = 0;
float angleYTotal = 0;
float angleZTotal = 0;

int angleXIndex_1 = 0;
int angleYIndex_1 = 0;
int angleZIndex_1 = 0;
float angleXTotal_1 = 0;
float angleYTotal_1 = 0;
float angleZTotal_1 = 0;

float lastAngleY = 0.0; // 이전 피치 값
float interpolatedAngleY = 0.0; // 보간된 피치 값

float lastAngleX = 0.0; // 이전 X 값
float interpolatedAngleX = 0.0; // 보간된 X 값

float lastAngleY_1 = 0.0; // 이전 피치 값
float interpolatedAngleY_1 = 0.0; // 보간된 피치 값

float lastAngleX_1 = 0.0; // 이전 X 값
float interpolatedAngleX_1 = 0.0; // 보간된 X 값

void setup() {

  Serial.begin(115200);

  pinMode(ReadPin1, INPUT);
  pinMode(ReadPin2, INPUT);

  pinMode(10, OUTPUT);

  servo20.attach(servoPin2);
  servo40.attach(servoPin4);
  servo50.attach(servoPin5);
  
  Wire.begin();

  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
  } else {
    Serial.println("MPU9250 is connected");
  }

  if (!myMPU9250_1.init()) {
    Serial.println("MPU9250_1 does not respond");
  } else {
    Serial.println("MPU9250_1 is connected");
  }
  
  if (!myMPU9250.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  } else {
    Serial.println("Magnetometer is connected");
  }

  if (!myMPU9250_1.initMagnetometer()) {
    Serial.println("Magnetometer_1 does not respond");
  } else {
    Serial.println("Magnetometer_1 is connected");
  }

  myMPU9250.autoOffsets();
  myMPU9250_1.autoOffsets();
  
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  
  myMPU9250.enableGyrDLPF();
  myMPU9250_1.enableGyrDLPF();
  
  myMPU9250.setGyrDLPF(MPU9250_DLPF_5);
  myMPU9250_1.setGyrDLPF(MPU9250_DLPF_5);
  //myMPU9250.setSampleRateDivider(5);

  myMPU9250.setSampleRateDivider(99);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_2000);
  
  myMPU9250_1.setSampleRateDivider(99);
  myMPU9250_1.setGyrRange(MPU9250_GYRO_RANGE_2000);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_16G);
  myMPU9250.enableAccDLPF(true);

  myMPU9250_1.setAccRange(MPU9250_ACC_RANGE_16G);
  myMPU9250_1.enableAccDLPF(true);
  
  myMPU9250.setAccDLPF(MPU9250_DLPF_5);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  myMPU9250_1.setAccDLPF(MPU9250_DLPF_5);
  myMPU9250_1.setMagOpMode(AK8963_CONT_MODE_100HZ);

  // 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    angleXReadings[i] = 0.0;
    angleYReadings[i] = 0.0;
    angleZReadings[i] = 0.0;
    angleXReadings_1[i] = 0.0;
    angleYReadings_1[i] = 0.0;
    angleZReadings_1[i] = 0.0;
  }

  delay(200);
}

void loop() {

  int currentReadPin1 = digitalRead(ReadPin1);
  
  
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gValue_1 = myMPU9250_1.getGValues();
  
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat gyr_1 = myMPU9250_1.getGyrValues();
  
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat magValue_1 = myMPU9250_1.getMagValues();
  
  xyzFloat angle = myMPU9250.getAngles();
  xyzFloat angle_1 = myMPU9250_1.getAngles();

  xyzFloat gyrRaw = myMPU9250.getGyrRawValues();
  xyzFloat gyrRaw_1 = myMPU9250_1.getGyrRawValues();
  
  xyzFloat corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();
  xyzFloat corrGyrRaw_1 = myMPU9250_1.getCorrectedGyrRawValues();

  // Remove gravity acceleration from acceleration values
  float alpha = 0.02;  // Smoothing factor
  pureAccX = alpha * pureAccX + (1 - alpha) * (gValue.x - gravityX);
  pureAccX_1 = alpha * pureAccX_1 + (1 - alpha) * (gValue_1.x - gravityX_1);
  
  pureAccY = alpha * pureAccY + (1 - alpha) * (gValue.y - gravityY);
  pureAccY_1 = alpha * pureAccY_1 + (1 - alpha) * (gValue_1.y - gravityY_1);
  
  pureAccZ = alpha * pureAccZ + (1 - alpha) * (gValue.z - gravityZ);
  pureAccZ_1 = alpha * pureAccZ_1 + (1 - alpha) * (gValue_1.z - gravityZ_1);

  // Update gravity values
  gravityX = gValue.x;
  gravityX_1 = gValue_1.x;
  
  gravityY = gValue.y;
  gravityY_1 = gValue_1.y;
  
  gravityZ = gValue.z;
  gravityZ_1 = gValue_1.z;

  float angleX = angle.x;
  float angleX_1 = angle_1.x;
  
  float angleY = angle.y;
  float angleY_1 = angle_1.y;
  
  float angleZ = calculateYaw(gValue.x, gValue.y, gValue.z); // 새로운 코드: Z축으로의 회전
  float angleZ_1 = calculateYaw(gValue_1.x, gValue_1.y, gValue_1.z);

  // 이동평균 필터 적용
  angleXTotal -= angleXReadings[angleXIndex];
  angleXTotal_1 -= angleXReadings_1[angleXIndex_1];
  
  angleYTotal -= angleYReadings[angleYIndex];
  angleYTotal_1 -= angleYReadings_1[angleYIndex_1];
  
  angleZTotal -= angleZReadings[angleZIndex];
  angleZTotal_1 -= angleZReadings_1[angleZIndex_1];

  angleXReadings[angleXIndex] = angleX;
  angleXReadings_1[angleXIndex_1] = angleX_1;
  
  angleYReadings[angleYIndex] = angleY;
  angleYReadings_1[angleYIndex_1] = angleY_1;
  
  angleZReadings[angleZIndex] = angleZ;
  angleZReadings_1[angleZIndex_1] = angleZ_1;

  angleXTotal += angleX;
  angleXTotal_1 += angleX_1;
  
  angleYTotal += angleY;
  angleYTotal_1 += angleY_1;
  
  angleZTotal += angleZ;
  angleZTotal_1 += angleZ_1;

  angleXIndex = (angleXIndex + 1) % numReadings;
  angleXIndex_1 = (angleXIndex_1 + 1) % numReadings;
  
  angleYIndex = (angleYIndex + 1) % numReadings;
  angleYIndex_1 = (angleYIndex_1 + 1) % numReadings;
  
  angleZIndex = (angleZIndex + 1) % numReadings;
  angleZIndex_1 = (angleZIndex_1 + 1) % numReadings;

  float smoothedAngleX = angleXTotal / numReadings;
  float smoothedAngleX_1 = angleXTotal_1 / numReadings;
  
  float smoothedAngleY = angleYTotal / numReadings;
  float smoothedAngleY_1 = angleYTotal_1 / numReadings;
  
  float smoothedAngleZ = angleZTotal / numReadings;
  float smoothedAngleZ_1 = angleZTotal_1 / numReadings;

  int angleXInt = static_cast<int>(smoothedAngleX);
  int angleXInt_1 = static_cast<int>(smoothedAngleX_1);
  
  int angleYInt = static_cast<int>(smoothedAngleY);
  int angleYInt_1 = static_cast<int>(smoothedAngleY_1);
  
  int angleZInt = static_cast<int>(smoothedAngleZ);
  int angleZInt_1 = static_cast<int>(smoothedAngleZ_1);


  // 보간된 피치 값 계산
  if (abs(angleYInt + 90 - lastAngleY) >= 2) {
    interpolatedAngleY = angleYInt + 90;
  } else {
    interpolatedAngleY = lastAngleY;
  }

    // 보간된 ROLL 값 계산
  if (abs(angleXInt + 90 - lastAngleX) >= 2) {
    interpolatedAngleX = angleXInt + 90;
  } else {
    interpolatedAngleX = lastAngleX;
  }

  if (abs(angleYInt_1 + 90 - lastAngleY_1) >= 2) {
    interpolatedAngleY_1 = angleYInt_1 + 90;
  } else {
    interpolatedAngleY_1 = lastAngleY_1;
  }

  lastAngleY = interpolatedAngleY;
  lastAngleX = interpolatedAngleX;
  lastAngleY_1 = interpolatedAngleY_1;

  int Servo2 = int (interpolatedAngleY_1);
  int Servo4 = int(interpolatedAngleX);
  int Servo5 = int(interpolatedAngleY);

  Servo2 = constrain(Servo2, 75, 145);
  Servo4 = constrain(Servo4, 1, 179);
  Servo5 = constrain(Servo5, 1, 179);

  Result2 = digitalRead(ReadPin2);

    if (currentReadPin1 != previousReadPin1) {
    // ReadPin 값이 변했을 때
    if (currentReadPin1 == 1) {
      // 0에서 1으로 변하는 순간
      
      servo20.write(Servo2);
      servo40.write(previousServo4);
      servo50.write(previousServo5);
      
      if (Result2 == 0) {
      digitalWrite(10, LOW);   
    } else if (Result2 == 1) {
      digitalWrite(10, HIGH); 
    }
    }
  }

  previousReadPin1 = currentReadPin1;

  if (currentReadPin1 == 0) {
    // 1에서 0으로 변하는 순간부터 Servo3 값을 유지하고 출력
    Servo4 = previousServo4;
    Servo5 = previousServo5;

      servo20.write(Servo2);
      servo40.write(Servo4);
      servo50.write(Servo5);
      if (Result2 == 0) {
      digitalWrite(10, LOW);   
    } else if (Result2 == 1) {
      digitalWrite(10, HIGH); 
    }
      
      
  } else {
      servo20.write(Servo2);
      servo40.write(Servo4);
      servo50.write(Servo5);
      if (Result2 == 0) {
      digitalWrite(10, LOW);   
    } else if (Result2 == 1) {
      digitalWrite(10, HIGH); 
    }
    
    previousServo4 = Servo4;
    previousServo5 = Servo5;
  }

  delay(5);
}

float calculateYaw(float accX, float accY, float accZ) {
  float pitch = atan2(accY, sqrt(accX * accX + accZ * accZ));
  float roll = atan2(-accX, accZ);
  float yaw = atan2(sin(roll) * cos(pitch), cos(roll) * cos(pitch));
  return yaw;
}



/*#include <MPU9250_WE.h>  //최종 버전 코드 **결국 I2C 써야함
#include <Wire.h>
#include <SoftwareSerial.h>

#define MPU9250_ADDR_0 0x68 //GND 핀 설정
#define MPU9250_ADDR_1 0x69 //3V3 핀 설정

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR_0);
MPU9250_WE myMPU9250_1 = MPU9250_WE(MPU9250_ADDR_1);

float gravityX = 0, gravityY = 0, gravityZ = 0;
float pureAccX = 0.0, pureAccY = 0.0, pureAccZ = 0.0;

float gravityX_1 = 0, gravityY_1 = 0, gravityZ_1 = 0;
float pureAccX_1 = 0.0, pureAccY_1 = 0.0, pureAccZ_1 = 0.0;

SoftwareSerial bluetooth(2, 3); // RX, TX
SoftwareSerial mySerial(10,9);  // RX, TX

int ReadPin1 = 11;
int ReadPin2 = 12;
const int ledPin = LED_BUILTIN;

int Result1 = 0;
int Result2 = 0;

const int numReadings = 20; // 이동평균 필터의 샘플 수
float angleXReadings[numReadings]; // 롤(Roll) 값을 저장할 배열
float angleXReadings_1[numReadings]; // 롤(Roll) 값을 저장할 배열
float angleYReadings[numReadings]; // 피치(Pitch) 값을 저장할 배열
float angleYReadings_1[numReadings]; // 피치(Pitch) 값을 저장할 배열
float angleZReadings[numReadings]; // 요(Yaw) 값을 저장할 배열
float angleZReadings_1[numReadings]; // 요(Yaw) 값을 저장할 배열
int angleXIndex = 0;
int angleYIndex = 0;
int angleZIndex = 0;
float angleXTotal = 0;
float angleYTotal = 0;
float angleZTotal = 0;

int angleXIndex_1 = 0;
int angleYIndex_1 = 0;
int angleZIndex_1 = 0;
float angleXTotal_1 = 0;
float angleYTotal_1 = 0;
float angleZTotal_1 = 0;

float lastAngleY = 0.0; // 이전 피치 값
float interpolatedAngleY = 0.0; // 보간된 피치 값

float lastAngleX = 0.0; // 이전 X 값
float interpolatedAngleX = 0.0; // 보간된 X 값

float lastAngleY_1 = 0.0; // 이전 피치 값
float interpolatedAngleY_1 = 0.0; // 보간된 피치 값

float lastAngleX_1 = 0.0; // 이전 X 값
float interpolatedAngleX_1 = 0.0; // 보간된 X 값

void setup() {

  Serial.begin(115200);
  bluetooth.begin(115200); // 블루투스 모듈 초기화

  pinMode(ReadPin1, INPUT);
  pinMode(ReadPin2, INPUT);
  pinMode(ledPin, OUTPUT);
  
  Wire.begin();

  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
  } else {
    Serial.println("MPU9250 is connected");
  }

  if (!myMPU9250_1.init()) {
    Serial.println("MPU9250_1 does not respond");
  } else {
    Serial.println("MPU9250_1 is connected");
  }
  
  if (!myMPU9250.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  } else {
    Serial.println("Magnetometer is connected");
  }

  if (!myMPU9250_1.initMagnetometer()) {
    Serial.println("Magnetometer_1 does not respond");
  } else {
    Serial.println("Magnetometer_1 is connected");
  }

  myMPU9250.autoOffsets();
  myMPU9250_1.autoOffsets();
  
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  
  myMPU9250.enableGyrDLPF();
  myMPU9250_1.enableGyrDLPF();
  
  myMPU9250.setGyrDLPF(MPU9250_DLPF_5);
  myMPU9250_1.setGyrDLPF(MPU9250_DLPF_5);
  //myMPU9250.setSampleRateDivider(5);

  myMPU9250.setSampleRateDivider(99);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_2000);
  
  myMPU9250_1.setSampleRateDivider(99);
  myMPU9250_1.setGyrRange(MPU9250_GYRO_RANGE_2000);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_16G);
  myMPU9250.enableAccDLPF(true);

  myMPU9250_1.setAccRange(MPU9250_ACC_RANGE_16G);
  myMPU9250_1.enableAccDLPF(true);
  
  myMPU9250.setAccDLPF(MPU9250_DLPF_5);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  myMPU9250_1.setAccDLPF(MPU9250_DLPF_5);
  myMPU9250_1.setMagOpMode(AK8963_CONT_MODE_100HZ);

  // 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    angleXReadings[i] = 0.0;
    angleYReadings[i] = 0.0;
    angleZReadings[i] = 0.0;
    angleXReadings_1[i] = 0.0;
    angleYReadings_1[i] = 0.0;
    angleZReadings_1[i] = 0.0;
  }

  delay(200);
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    bluetooth.write(c);
  }

  // 블루투스 모듈 입력을 시리얼로 출력
  if (bluetooth.available()) {
    char c = bluetooth.read();
    Serial.write(c);
  }

  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gValue_1 = myMPU9250_1.getGValues();
  
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat gyr_1 = myMPU9250_1.getGyrValues();
  
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat magValue_1 = myMPU9250_1.getMagValues();
  
  xyzFloat angle = myMPU9250.getAngles();
  xyzFloat angle_1 = myMPU9250_1.getAngles();

  xyzFloat gyrRaw = myMPU9250.getGyrRawValues();
  xyzFloat gyrRaw_1 = myMPU9250_1.getGyrRawValues();
  
  xyzFloat corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();
  xyzFloat corrGyrRaw_1 = myMPU9250_1.getCorrectedGyrRawValues();

  // Remove gravity acceleration from acceleration values
  float alpha = 0.02;  // Smoothing factor
  pureAccX = alpha * pureAccX + (1 - alpha) * (gValue.x - gravityX);
  pureAccX_1 = alpha * pureAccX_1 + (1 - alpha) * (gValue_1.x - gravityX_1);
  
  pureAccY = alpha * pureAccY + (1 - alpha) * (gValue.y - gravityY);
  pureAccY_1 = alpha * pureAccY_1 + (1 - alpha) * (gValue_1.y - gravityY_1);
  
  pureAccZ = alpha * pureAccZ + (1 - alpha) * (gValue.z - gravityZ);
  pureAccZ_1 = alpha * pureAccZ_1 + (1 - alpha) * (gValue_1.z - gravityZ_1);

  // Update gravity values
  gravityX = gValue.x;
  gravityX_1 = gValue_1.x;
  
  gravityY = gValue.y;
  gravityY_1 = gValue_1.y;
  
  gravityZ = gValue.z;
  gravityZ_1 = gValue_1.z;

  float angleX = angle.x;
  float angleX_1 = angle_1.x;
  
  float angleY = angle.y;
  float angleY_1 = angle_1.y;
  
  float angleZ = calculateYaw(gValue.x, gValue.y, gValue.z); // 새로운 코드: Z축으로의 회전
  float angleZ_1 = calculateYaw(gValue_1.x, gValue_1.y, gValue_1.z);

  // 이동평균 필터 적용
  angleXTotal -= angleXReadings[angleXIndex];
  angleXTotal_1 -= angleXReadings_1[angleXIndex_1];
  
  angleYTotal -= angleYReadings[angleYIndex];
  angleYTotal_1 -= angleYReadings_1[angleYIndex_1];
  
  angleZTotal -= angleZReadings[angleZIndex];
  angleZTotal_1 -= angleZReadings_1[angleZIndex_1];

  angleXReadings[angleXIndex] = angleX;
  angleXReadings_1[angleXIndex_1] = angleX_1;
  
  angleYReadings[angleYIndex] = angleY;
  angleYReadings_1[angleYIndex_1] = angleY_1;
  
  angleZReadings[angleZIndex] = angleZ;
  angleZReadings_1[angleZIndex_1] = angleZ_1;

  angleXTotal += angleX;
  angleXTotal_1 += angleX_1;
  
  angleYTotal += angleY;
  angleYTotal_1 += angleY_1;
  
  angleZTotal += angleZ;
  angleZTotal_1 += angleZ_1;

  angleXIndex = (angleXIndex + 1) % numReadings;
  angleXIndex_1 = (angleXIndex_1 + 1) % numReadings;
  
  angleYIndex = (angleYIndex + 1) % numReadings;
  angleYIndex_1 = (angleYIndex_1 + 1) % numReadings;
  
  angleZIndex = (angleZIndex + 1) % numReadings;
  angleZIndex_1 = (angleZIndex_1 + 1) % numReadings;

  float smoothedAngleX = angleXTotal / numReadings;
  float smoothedAngleX_1 = angleXTotal_1 / numReadings;
  
  float smoothedAngleY = angleYTotal / numReadings;
  float smoothedAngleY_1 = angleYTotal_1 / numReadings;
  
  float smoothedAngleZ = angleZTotal / numReadings;
  float smoothedAngleZ_1 = angleZTotal_1 / numReadings;

  int angleXInt = static_cast<int>(smoothedAngleX);
  int angleXInt_1 = static_cast<int>(smoothedAngleX_1);
  
  int angleYInt = static_cast<int>(smoothedAngleY);
  int angleYInt_1 = static_cast<int>(smoothedAngleY_1);
  
  int angleZInt = static_cast<int>(smoothedAngleZ);
  int angleZInt_1 = static_cast<int>(smoothedAngleZ_1);


  // 보간된 피치 값 계산
  if (abs(angleYInt + 90 - lastAngleY) >= 2) {
    interpolatedAngleY = angleYInt + 90;
  } else {
    interpolatedAngleY = lastAngleY;
  }

    // 보간된 ROLL 값 계산
  if (abs(angleXInt + 90 - lastAngleX) >= 2) {
    interpolatedAngleX = angleXInt + 90;
  } else {
    interpolatedAngleX = lastAngleX;
  }

  if (abs(angleYInt_1 + 90 - lastAngleY_1) >= 2) {
    interpolatedAngleY_1 = angleYInt_1 + 90;
  } else {
    interpolatedAngleY_1 = lastAngleY_1;
  }

  lastAngleY = interpolatedAngleY;
  lastAngleX = interpolatedAngleX;
  lastAngleY_1 = interpolatedAngleY_1;

  int Servo2 = int (interpolatedAngleY_1);
  int Servo4 = int(interpolatedAngleX);
  int Servo5 = int(interpolatedAngleY);
  
  //Serial.println(angleXInt); // Angle x
  //Serial.print(",");
  //Serial.print(Servo1); 
  //Serial.print(",");

  Serial.print(Servo2); 
  Serial.print(",");
  //Serial.print(Servo3); 
  //Serial.print(",");
  Serial.print(Servo4); 
  Serial.print(",");
  Serial.println(Servo5); 

  Result1 = digitalRead(ReadPin1);
  Result2 = digitalRead(ReadPin2);
  Serial.print(Result1);
  Serial.print(",");
  Serial.println(Result2);
  //Serial.println("********************************************");

  delay(100);
}

float calculateYaw(float accX, float accY, float accZ) {
  float pitch = atan2(accY, sqrt(accX * accX + accZ * accZ));
  float roll = atan2(-accX, accZ);
  float yaw = atan2(sin(roll) * cos(pitch), cos(roll) * cos(pitch));
  return yaw;
}

*/

/*#include <MPU9250_WE.h>  //최종 버전 코드 였었음
#include <Wire.h>
#include <SoftwareSerial.h>

#define MPU9250_ADDR_0 0x68 //GND 핀 설정
#define MPU9250_ADDR_1 0x69 //3V3 핀 설정

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR_0);
MPU9250_WE myMPU9250_1 = MPU9250_WE(MPU9250_ADDR_1);

float gravityX = 0, gravityY = 0, gravityZ = 0;
float pureAccX = 0.0, pureAccY = 0.0, pureAccZ = 0.0;

float gravityX_1 = 0, gravityY_1 = 0, gravityZ_1 = 0;
float pureAccX_1 = 0.0, pureAccY_1 = 0.0, pureAccZ_1 = 0.0;

SoftwareSerial bluetooth(0, 1); // RX, TX

int ReadPin1 = 9;
int ReadPin2 = 10;

int Result1 = 0;
int Result2 = 0;

const int numReadings = 10; // 이동평균 필터의 샘플 수
float angleXReadings[numReadings]; // 롤(Roll) 값을 저장할 배열
float angleXReadings_1[numReadings]; // 롤(Roll) 값을 저장할 배열
float angleYReadings[numReadings]; // 피치(Pitch) 값을 저장할 배열
float angleYReadings_1[numReadings]; // 피치(Pitch) 값을 저장할 배열
float angleZReadings[numReadings]; // 요(Yaw) 값을 저장할 배열
float angleZReadings_1[numReadings]; // 요(Yaw) 값을 저장할 배열
int angleXIndex = 0;
int angleYIndex = 0;
int angleZIndex = 0;
float angleXTotal = 0;
float angleYTotal = 0;
float angleZTotal = 0;

int angleXIndex_1 = 0;
int angleYIndex_1 = 0;
int angleZIndex_1 = 0;
float angleXTotal_1 = 0;
float angleYTotal_1 = 0;
float angleZTotal_1 = 0;

float lastAngleY = 0.0; // 이전 피치 값
float interpolatedAngleY = 0.0; // 보간된 피치 값

float lastAngleY_1 = 0.0; // 이전 피치 값
float interpolatedAngleY_1 = 0.0; // 보간된 피치 값

void setup() {

  Serial.begin(115200);
  pinMode(ReadPin1, INPUT);
  pinMode(ReadPin2, INPUT);
  
  bluetooth.begin(115200); // 블루투스 모듈 초기화
  Wire.begin();
  if (!myMPU9250.init()) {
    //Serial.println("MPU9250 does not respond");
  } else {
    //Serial.println("MPU9250 is connected");
  }

  if (!myMPU9250_1.init()) {
    //Serial.println("MPU9250 does not respond");
  } else {
    //Serial.println("MPU9250 is connected");
  }
  
  if (!myMPU9250.initMagnetometer()) {
    //Serial.println("Magnetometer does not respond");
  } else {
    //Serial.println("Magnetometer is connected");
  }

  if (!myMPU9250_1.initMagnetometer()) {
    //Serial.println("Magnetometer does not respond");
  } else {
    //Serial.println("Magnetometer is connected");
  }

  myMPU9250.autoOffsets();
  myMPU9250_1.autoOffsets();
  
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  
  myMPU9250.enableGyrDLPF();
  myMPU9250_1.enableGyrDLPF();
  
  myMPU9250.setGyrDLPF(MPU9250_DLPF_5);
  myMPU9250_1.setGyrDLPF(MPU9250_DLPF_5);
  //myMPU9250.setSampleRateDivider(5);

  myMPU9250.setSampleRateDivider(99);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_2000);
  
  myMPU9250_1.setSampleRateDivider(99);
  myMPU9250_1.setGyrRange(MPU9250_GYRO_RANGE_2000);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_16G);
  myMPU9250.enableAccDLPF(true);

  myMPU9250_1.setAccRange(MPU9250_ACC_RANGE_16G);
  myMPU9250_1.enableAccDLPF(true);
  
  myMPU9250.setAccDLPF(MPU9250_DLPF_5);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  myMPU9250_1.setAccDLPF(MPU9250_DLPF_5);
  myMPU9250_1.setMagOpMode(AK8963_CONT_MODE_100HZ);

  // 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    angleXReadings[i] = 0.0;
    angleYReadings[i] = 0.0;
    angleZReadings[i] = 0.0;
    angleXReadings_1[i] = 0.0;
    angleYReadings_1[i] = 0.0;
    angleZReadings_1[i] = 0.0;
  }

  delay(200);
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    bluetooth.write(c);
  }

  // 블루투스 모듈 입력을 시리얼로 출력
  if (bluetooth.available()) {
    char c = bluetooth.read();
    Serial.write(c);
  }

  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gValue_1 = myMPU9250_1.getGValues();
  
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat gyr_1 = myMPU9250_1.getGyrValues();
  
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat magValue_1 = myMPU9250_1.getMagValues();
  
  xyzFloat angle = myMPU9250.getAngles();
  xyzFloat angle_1 = myMPU9250_1.getAngles();

  xyzFloat gyrRaw = myMPU9250.getGyrRawValues();
  xyzFloat gyrRaw_1 = myMPU9250_1.getGyrRawValues();
  
  xyzFloat corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();
  xyzFloat corrGyrRaw_1 = myMPU9250_1.getCorrectedGyrRawValues();

  // Remove gravity acceleration from acceleration values
  float alpha = 0.02;  // Smoothing factor
  pureAccX = alpha * pureAccX + (1 - alpha) * (gValue.x - gravityX);
  pureAccX_1 = alpha * pureAccX_1 + (1 - alpha) * (gValue_1.x - gravityX_1);
  
  pureAccY = alpha * pureAccY + (1 - alpha) * (gValue.y - gravityY);
  pureAccY_1 = alpha * pureAccY_1 + (1 - alpha) * (gValue_1.y - gravityY_1);
  
  pureAccZ = alpha * pureAccZ + (1 - alpha) * (gValue.z - gravityZ);
  pureAccZ_1 = alpha * pureAccZ_1 + (1 - alpha) * (gValue_1.z - gravityZ_1);

  // Update gravity values
  gravityX = gValue.x;
  gravityX_1 = gValue_1.x;
  
  gravityY = gValue.y;
  gravityY_1 = gValue_1.y;
  
  gravityZ = gValue.z;
  gravityZ_1 = gValue_1.z;

  float angleX = angle.x;
  float angleX_1 = angle_1.x;
  
  float angleY = angle.y;
  float angleY_1 = angle_1.y;
  
  float angleZ = calculateYaw(gValue.x, gValue.y, gValue.z); // 새로운 코드: Z축으로의 회전
  float angleZ_1 = calculateYaw(gValue_1.x, gValue_1.y, gValue_1.z);

  // 이동평균 필터 적용
  angleXTotal -= angleXReadings[angleXIndex];
  angleXTotal_1 -= angleXReadings_1[angleXIndex_1];
  
  angleYTotal -= angleYReadings[angleYIndex];
  angleYTotal_1 -= angleYReadings_1[angleYIndex_1];
  
  angleZTotal -= angleZReadings[angleZIndex];
  angleZTotal_1 -= angleZReadings_1[angleZIndex_1];

  angleXReadings[angleXIndex] = angleX;
  angleXReadings_1[angleXIndex_1] = angleX_1;
  
  angleYReadings[angleYIndex] = angleY;
  angleYReadings_1[angleYIndex_1] = angleY_1;
  
  angleZReadings[angleZIndex] = angleZ;
  angleZReadings_1[angleZIndex_1] = angleZ_1;

  angleXTotal += angleX;
  angleXTotal_1 += angleX_1;
  
  angleYTotal += angleY;
  angleYTotal_1 += angleY_1;
  
  angleZTotal += angleZ;
  angleZTotal_1 += angleZ_1;

  angleXIndex = (angleXIndex + 1) % numReadings;
  angleXIndex_1 = (angleXIndex_1 + 1) % numReadings;
  
  angleYIndex = (angleYIndex + 1) % numReadings;
  angleYIndex_1 = (angleYIndex_1 + 1) % numReadings;
  
  angleZIndex = (angleZIndex + 1) % numReadings;
  angleZIndex_1 = (angleZIndex_1 + 1) % numReadings;

  float smoothedAngleX = angleXTotal / numReadings;
  float smoothedAngleX_1 = angleXTotal_1 / numReadings;
  
  float smoothedAngleY = angleYTotal / numReadings;
  float smoothedAngleY_1 = angleYTotal_1 / numReadings;
  
  float smoothedAngleZ = angleZTotal / numReadings;
  float smoothedAngleZ_1 = angleZTotal_1 / numReadings;

  int angleXInt = static_cast<int>(smoothedAngleX);
  int angleXInt_1 = static_cast<int>(smoothedAngleX_1);
  
  int angleYInt = static_cast<int>(smoothedAngleY);
  int angleYInt_1 = static_cast<int>(smoothedAngleY_1);
  
  int angleZInt = static_cast<int>(smoothedAngleZ);
  int angleZInt_1 = static_cast<int>(smoothedAngleZ_1);

  // 보간된 피치 값 계산
  if (abs(angleYInt + 90 - lastAngleY) >= 2) {
    interpolatedAngleY = angleYInt + 90;
  } else {
    interpolatedAngleY = lastAngleY;
  }

  if (abs(angleYInt_1 + 90 - lastAngleY_1) >= 2) {
    interpolatedAngleY_1 = angleYInt_1 + 90;
  } else {
    interpolatedAngleY_1 = lastAngleY_1;
  }

  //Serial.println(angleXInt); // Angle x
  //Serial.print(",");
  Serial.print(interpolatedAngleY); // Angle y
  Serial.print(",");
  Serial.print(interpolatedAngleY_1); // Angle y
  Serial.print(",");
  //Serial.println(interpolatedAngleZ); // Angle z
 

  lastAngleY = interpolatedAngleY;


  Result1 = digitalRead(ReadPin1);
  Result2 = digitalRead(ReadPin2);
  Serial.print(Result1);
  Serial.print(",");
  Serial.println(Result2);
  //Serial.println("********************************************");

  delay(100);
}

float calculateYaw(float accX, float accY, float accZ) {
  float pitch = atan2(accY, sqrt(accX * accX + accZ * accZ));
  float roll = atan2(-accX, accZ);
  float yaw = atan2(sin(roll) * cos(pitch), cos(roll) * cos(pitch));
  return yaw;
}

*/

/* #include <MPU9250_WE.h>  //최종 버전 코드였었음
#include <Wire.h>
#include <SoftwareSerial.h>

#define MPU9250_ADDR 0x68 //GND 핀 설정

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

float gravityX = 0, gravityY = 0, gravityZ = 0;
float pureAccX = 0.0, pureAccY = 0.0, pureAccZ = 0.0;

SoftwareSerial bluetooth(0, 1); // RX, TX

#define sample_num_mdate  5000

int ReadPin1 = 9;
int ReadPin2 = 10;

int Result1 = 0;
int Result2 = 0;

const int numReadings = 10; // 이동평균 필터의 샘플 수
float angleXReadings[numReadings]; // 롤(Roll) 값을 저장할 배열
float angleYReadings[numReadings]; // 피치(Pitch) 값을 저장할 배열
float angleZReadings[numReadings]; // 요(Yaw) 값을 저장할 배열
int angleXIndex = 0;
int angleYIndex = 0;
int angleZIndex = 0;
float angleXTotal = 0;
float angleYTotal = 0;
float angleZTotal = 0;

float lastAngleY = 0.0; // 이전 피치 값
float interpolatedAngleY = 0.0; // 보간된 피치 값

void setup() {

  Serial.begin(115200);
  pinMode(ReadPin1, INPUT);
  pinMode(ReadPin2, INPUT);
  
  bluetooth.begin(115200); // 블루투스 모듈 초기화
  Wire.begin();
  if (!myMPU9250.init()) {
    //Serial.println("MPU9250 does not respond");
  } else {
    //Serial.println("MPU9250 is connected");
  }
  if (!myMPU9250.initMagnetometer()) {
    //Serial.println("Magnetometer does not respond");
  } else {
    //Serial.println("Magnetometer is connected");
  }

  myMPU9250.autoOffsets();
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_5);
  //myMPU9250.setSampleRateDivider(5);

  myMPU9250.setSampleRateDivider(99);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_2000);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_16G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_5);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  // 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    angleXReadings[i] = 0.0;
    angleYReadings[i] = 0.0;
    angleZReadings[i] = 0.0;
  }

  delay(200);
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    bluetooth.write(c);
  }

  // 블루투스 모듈 입력을 시리얼로 출력
  if (bluetooth.available()) {
    char c = bluetooth.read();
    Serial.write(c);
  }

  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat angle = myMPU9250.getAngles();

  xyzFloat gyrRaw = myMPU9250.getGyrRawValues();
  xyzFloat corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();

  // Remove gravity acceleration from acceleration values
  float alpha = 0.02;  // Smoothing factor
  pureAccX = alpha * pureAccX + (1 - alpha) * (gValue.x - gravityX);
  pureAccY = alpha * pureAccY + (1 - alpha) * (gValue.y - gravityY);
  pureAccZ = alpha * pureAccZ + (1 - alpha) * (gValue.z - gravityZ);

  // Update gravity values
  gravityX = gValue.x;
  gravityY = gValue.y;
  gravityZ = gValue.z;

  float angleX = angle.x;
  float angleY = angle.y;
  float angleZ = calculateYaw(gValue.x, gValue.y, gValue.z); // 새로운 코드: Z축으로의 회전

  // 이동평균 필터 적용
  angleXTotal -= angleXReadings[angleXIndex];
  angleYTotal -= angleYReadings[angleYIndex];
  angleZTotal -= angleZReadings[angleZIndex];

  angleXReadings[angleXIndex] = angleX;
  angleYReadings[angleYIndex] = angleY;
  angleZReadings[angleZIndex] = angleZ;

  angleXTotal += angleX;
  angleYTotal += angleY;
  angleZTotal += angleZ;

  angleXIndex = (angleXIndex + 1) % numReadings;
  angleYIndex = (angleYIndex + 1) % numReadings;
  angleZIndex = (angleZIndex + 1) % numReadings;

  float smoothedAngleX = angleXTotal / numReadings;
  float smoothedAngleY = angleYTotal / numReadings;
  float smoothedAngleZ = angleZTotal / numReadings;

  int angleXInt = static_cast<int>(smoothedAngleX);
  int angleYInt = static_cast<int>(smoothedAngleY);
  int angleZInt = static_cast<int>(smoothedAngleZ);

  // 보간된 피치 값 계산
  if (abs(angleYInt + 90 - lastAngleY) >= 2) {
    interpolatedAngleY = angleYInt + 90;
  } else {
    interpolatedAngleY = lastAngleY;
  }

  //Serial.println(angleXInt); // Angle x
  //Serial.print(",");
  Serial.print(interpolatedAngleY); // Angle y
  Serial.print(",");
  //Serial.println(angleZInt); // Angle z

  lastAngleY = interpolatedAngleY;


  Result1 = digitalRead(ReadPin1);
  Result2 = digitalRead(ReadPin2);
  Serial.print(Result1);
  Serial.print(",");
  Serial.println(Result2);
  //Serial.println("********************************************");

  delay(100);
}

float calculateYaw(float accX, float accY, float accZ) {
  float pitch = atan2(accY, sqrt(accX * accX + accZ * accZ));
  float roll = atan2(-accX, accZ);
  float yaw = atan2(sin(roll) * cos(pitch), cos(roll) * cos(pitch));
  return yaw;
}

*/


/*#include <MPU9250_WE.h>  //원래 최종 장갑 코드
#include <Wire.h>
#include <SoftwareSerial.h>

#define MPU9250_ADDR 0x68

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

float gravityX = 0, gravityY = 0, gravityZ = 0;
float pureAccX = 0.0, pureAccY = 0.0, pureAccZ = 0.0;

SoftwareSerial bluetooth(0, 1); // RX, TX

#define sample_num_mdate  5000

int ReadPin = 13;

int Result = 0;

void setup() {
  
  Serial.begin(115200);
  pinMode(ReadPin, INPUT);
  bluetooth.begin(115200); // 블루투스 모듈 초기화
  Wire.begin();
  if (!myMPU9250.init()) {
    //Serial.println("MPU9250 does not respond");
  } else {
    //Serial.println("MPU9250 is connected");
  }
  if (!myMPU9250.initMagnetometer()) {
    //Serial.println("Magnetometer does not respond");
  } else {
    //Serial.println("Magnetometer is connected");
  }

  myMPU9250.autoOffsets();
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_5);
  //myMPU9250.setSampleRateDivider(5);
  myMPU9250.setSampleRateDivider(99);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_1000);
  
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  delay(200);
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    bluetooth.write(c);
  }

  // 블루투스 모듈 입력을 시리얼로 출력
  if (bluetooth.available()) {
    char c = bluetooth.read();
    Serial.write(c);
  }

  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat angle = myMPU9250.getAngles();

  xyzFloat gyrRaw = myMPU9250.getGyrRawValues();
  xyzFloat corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();

  // Remove gravity acceleration from acceleration values
  float alpha = 0.02;  // Smoothing factor
  pureAccX = alpha * pureAccX + (1 - alpha) * (gValue.x - gravityX);
  pureAccY = alpha * pureAccY + (1 - alpha) * (gValue.y - gravityY);
  pureAccZ = alpha * pureAccZ + (1 - alpha) * (gValue.z - gravityZ);

  // Update gravity values
  gravityX = gValue.x;
  gravityY = gValue.y;
  gravityZ = gValue.z;

  float angleX = angle.x;
  int angleXInt = static_cast<int>(angleX);
  float angleY = angle.y;
  int angleYInt = static_cast<int>(angleY);
  float angleZ = angle.z;
  int angleZInt = static_cast<int>(angleZ);

/*
  Serial.print(pureAccX); // Acceleration in g (x, y, z)
  Serial.print(",");
  Serial.print(pureAccY);
  Serial.print(",");
  Serial.print(pureAccZ);
  Serial.print(",");

  Serial.print(gyr.x); // Gyroscope Data in degrees/s (x,y,z)
  Serial.print(",");
  Serial.print(gyr.y);
  Serial.print(",");
  Serial.print(gyr.z);
  Serial.print(",");

  Serial.print(magValue.x);  // Magnetometer Data in µTesla
  Serial.print(",");
  Serial.print(magValue.y);
  Serial.print(",");
  Serial.print(magValue.z);
  Serial.print(",");

  Serial.print(angleXInt); // Angle x
  Serial.print(",");
  Serial.print(angleYInt); // Angle y
  Serial.print(",");
  Serial.println(angleZInt); // Angle z
  
  Result = digitalRead(ReadPin);
  //Serial.println(Result);  // 초음파 on / off

  //Serial.println("********************************************");

  delay(100);
}*/
