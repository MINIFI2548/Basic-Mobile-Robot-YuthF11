#include <Arduino.h>
#include "Motor.h"
#include "config.h"

#define KP_SPEED_CONTROLL 0.03

Motor frontLeftMotor  ( FORNT_LEFT_MOTOR_PWM, FORNT_LEFT_MOTOR_IN1, FORNT_LEFT_MOTOR_IN2, 
                        FORNT_LEFT_MOTOR_ECA, FORNT_LEFT_MOTOR_ECB  );
Motor frontRightMotor (  FORNT_RIGHT_MOTOR_PWM, FORNT_RIGHT_MOTOR_IN1, FORNT_RIGHT_MOTOR_IN2, 
                        FORNT_RIGHT_MOTOR_ECA, FORNT_RIGHT_MOTOR_ECB  );
Motor backLeftMotor   ( BACK_LEFT_MOTOR_PWM, BACK_LEFT_MOTOR_IN1, BACK_LEFT_MOTOR_IN2, 
                        BACK_LEFT_MOTOR_ECA, BACK_LEFT_MOTOR_ECB  );
Motor backRightMotor  (  BACK_RIGHT_MOTOR_PWM, BACK_RIGHT_MOTOR_IN1, BACK_RIGHT_MOTOR_IN2, 
                        BACK_RIGHT_MOTOR_ECA, BACK_RIGHT_MOTOR_ECB  );

volatile double curruntPosiX, curruntPosiY, yaw;
long lastTime;

void upDatePosition(){
  curruntPosiY = RADIUS_OF_WHEELS * (frontLeftMotor.getEncoder() + frontRightMotor.getEncoder() + backLeftMotor.getEncoder() + backRightMotor.getEncoder()) * PI / 78000;
  curruntPosiX = RADIUS_OF_WHEELS * (frontLeftMotor.getEncoder() - frontRightMotor.getEncoder() - backLeftMotor.getEncoder() + backRightMotor.getEncoder()) * PI / 78000; 
  yaw = RADIUS_OF_WHEELS * 180 * (frontRightMotor.getPosition() - frontLeftMotor.getPosition() - backLeftMotor.getPosition() + backRightMotor.getPosition()) / (PI * 4 * (DISTANCE_FROM_WHEELS_X + DISTANCE_FROM_WHEELS_Y));
}

void speedControllConfig(){
  frontLeftMotor.setOffset();
  frontRightMotor.setOffset();
  backLeftMotor.setOffset();
  backRightMotor.setOffset();
}

void speedControll(float FLPulse, float FRPulse, float BLPulse, float BRPulse){ 
  static long lastTime;
  float maxPulse = FLPulse;
  if(millis() - lastTime >= 5){
    Motor * mainMotor = &frontLeftMotor;
    if(abs(FRPulse) > abs(maxPulse)){
      maxPulse = FRPulse;
      mainMotor = &frontRightMotor;
    }
    if(abs(BLPulse) > abs(maxPulse)){
      maxPulse = BLPulse;
      mainMotor = &backLeftMotor;
    }
    if(abs(BRPulse) > abs(maxPulse)){
      maxPulse = BRPulse;
      mainMotor = &backRightMotor;
    }

    float errorFL = (FLPulse * (mainMotor->getEncoder() - mainMotor->getOffset())) - (maxPulse * (frontLeftMotor.getEncoder() - frontLeftMotor.getOffset()));
    float errorFR = (FRPulse * (mainMotor->getEncoder() - mainMotor->getOffset())) - (maxPulse * (frontRightMotor.getEncoder() - frontRightMotor.getOffset()));
    float errorBL = (BLPulse * (mainMotor->getEncoder() - mainMotor->getOffset())) - (maxPulse * (backLeftMotor.getEncoder() - backLeftMotor.getOffset()));
    float errorBR = (BRPulse * (mainMotor->getEncoder() - mainMotor->getOffset())) - (maxPulse * (backRightMotor.getEncoder() - backRightMotor.getOffset()));

    float outFL = FLPulse + ((abs(maxPulse + 1) - abs(maxPulse)) * errorFL * KP_SPEED_CONTROLL);
    float outFR = FRPulse + ((abs(maxPulse + 1) - abs(maxPulse)) * errorFR * KP_SPEED_CONTROLL);
    float outBL = BLPulse + ((abs(maxPulse + 1) - abs(maxPulse)) * errorBL * KP_SPEED_CONTROLL);
    float outBR = BRPulse + ((abs(maxPulse + 1) - abs(maxPulse)) * errorBR * KP_SPEED_CONTROLL);

    frontLeftMotor.setPulse(outFL);
    frontRightMotor.setPulse(outFR);
    backLeftMotor.setPulse(outBL);
    backRightMotor.setPulse(outBR);
    lastTime = millis();

    // Serial.print(outFL);Serial.print(",");
    // Serial.print(outFR);Serial.print(",");
    // Serial.print(outBL);Serial.print(",");
    // Serial.println(outBR);
  }
}

void ao(){
  frontLeftMotor.setPulse(0);
  frontRightMotor.setPulse(0);
  backLeftMotor.setPulse(0);
  backRightMotor.setPulse(0);
  delay(100);
}

void wait_OK() {
  pinMode(OK_BUTTOM_PIN, INPUT_PULLUP);
  while(digitalRead(OK_BUTTOM_PIN)) {};
}

void showInfo(){
      frontLeftMotor.updateVelocity();
      frontRightMotor.updateVelocity();
      backLeftMotor.updateVelocity();
      backRightMotor.updateVelocity();
      if(millis() - lastTime > 250){
      Serial.print("Position X : ");
      Serial.print(curruntPosiX);
      Serial.print("\tPosition Y : ");
      Serial.print(curruntPosiY);
      Serial.print("\tYax : ");
      Serial.println(yaw);

      Serial.print("FL Velocity : ");
      Serial.print(frontLeftMotor.getVelocity());
      Serial.print("\tFR Velocity : ");
      Serial.println(frontRightMotor.getVelocity());
      Serial.print("BL Velocity: ");
      Serial.print(backLeftMotor.getVelocity());
      Serial.print("\tBR Velocity : ");
      Serial.println(backRightMotor.getVelocity());
      Serial.println("\n###############################################################\n");
      lastTime = millis();
    }
}

void move(double positionX, double positionY){
  upDatePosition();
  bool ignouX = positionX == 0;
  bool ignouY = positionY == 0;
  double targetPositionX = positionX + curruntPosiX;
  double targetPositionY = positionY + curruntPosiY;
  double targetYaw = 0;
  
  double deltaPositionX = (long)(positionX);
  double deltaPositionY = (long)(positionY);


  // mode
  int mode = (deltaPositionX > 0) << 1 | (deltaPositionY > 0);

  speedControllConfig();
  static long time;
  while (true){
    double deltaYaw = -yaw;
    double targetPosiFL = ((positionY + positionX) / 24.504422698 * ENCODER_PER_REVOLUTION) - (deltaYaw * 206.175); 
    double targetPosiFR = ((positionY - positionX) / 24.504422698 * ENCODER_PER_REVOLUTION) + (deltaYaw * 206.175); 
    double targetPosiBL = ((positionY - positionX) / 24.504422698 * ENCODER_PER_REVOLUTION) - (deltaYaw * 206.175); 
    double targetPosiBR = ((positionY + positionX) / 24.504422698 * ENCODER_PER_REVOLUTION) + (deltaYaw * 206.175); 

    double maxTargetPosi = targetPosiFL; 
    maxTargetPosi = (abs(targetPosiFR) > abs(maxTargetPosi)) ? targetPosiFR : maxTargetPosi;
    maxTargetPosi = (abs(targetPosiBL) > abs(maxTargetPosi)) ? targetPosiBL : maxTargetPosi;
    maxTargetPosi = (abs(targetPosiBR) > abs(maxTargetPosi)) ? targetPosiBR : maxTargetPosi;

    double velocityFL = 100 * abs(targetPosiFL / maxTargetPosi) * ((abs(targetPosiFL + 1) - abs(targetPosiFL)));
    double velocityFR = 100 * abs(targetPosiFR / maxTargetPosi) * ((abs(targetPosiFR + 1) - abs(targetPosiFR)));
    double velocityBL = 100 * abs(targetPosiBL / maxTargetPosi) * ((abs(targetPosiBL + 1) - abs(targetPosiBL)));
    double velocityBR = 100 * abs(targetPosiBR / maxTargetPosi) * ((abs(targetPosiBR + 1) - abs(targetPosiBR)));

    upDatePosition();
    bool stateX =   (((mode & 0b10) == 0b00) && ((curruntPosiX <= targetPositionX))) || \
                    (((mode & 0b10) == 0b10) && ((curruntPosiX >=  targetPositionX))) || ignouX;
    bool stateY =   (((mode & 0b01) == 0b00) && ((curruntPosiY <= targetPositionY))) ||\
                    (((mode & 0b01) == 0b01) && ((curruntPosiY >= targetPositionY))) || ignouY;
    if(stateX && stateY){
      ao();
      break;
    }
    speedControll(velocityFL, velocityFR, velocityBL, velocityBR);
    showInfo();
  }
}

void circle() {
  speedControllConfig();
  while(yaw > -350){
    upDatePosition();
    speedControll(-75, 75, 150, -150);
    showInfo();
  }
  ao();
}

void house() {
  move(25,25);
  delay(200);
  move(25,-25);
  delay(200);
  move(-50,0);
  delay(200);
  move(0,-50);
  delay(200);
  move(50,0);
  delay(200);
  move(0,50);
  delay(200);
}

void umbrella(){
  move(-25,0);
  delay(200);
  move(25,50);
  delay(200);
  move(25,-50);
  delay(200);
  move(-25,0);
  delay(200);
  move(0,-50);
  delay(200);
  move(-10,0);
  delay(200);
  move(0, 10);
}

void triangle(){
  move(50,50);
  delay(200);
  move(50,-50);
  delay(200);
  move(-100,0);
  delay(200);
}

void star() {
  move(25,50);
  delay(200);
  move(25,-50);
  delay(200);
  move(-50,25);
  delay(200);
  move(50,0);
  delay(200);
  move(-50,-25);
  delay(200);
}
void setup() {
  frontLeftMotor.init(0);
  frontRightMotor.init(1);
  backLeftMotor.init(2);
  backRightMotor.init(3);
  // wait_OK();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(OK_BUTTOM_PIN, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  if(Serial.available() > 0) {
    byte buffer = Serial.read();
    Serial.println(buffer);
    switch (buffer)
    {
    case 'h':
      Serial.println("house");
      house();
      break;
    case 'u':
      Serial.println("umbrella");
      umbrella();
      break;
    case 's':
      Serial.println("star");
      star();
      break;
    case 'o':
      Serial.println("circle");
      circle();
      break;
    case 't':
      Serial.println("circle");
      triangle();
      break;
    default:
      break;
    }
  }
  upDatePosition();
  showInfo();
}