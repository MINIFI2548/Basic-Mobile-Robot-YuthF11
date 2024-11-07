#include <Arduino.h>
#include <Motor.h>

Motor *Motor::Encoder[] = {nullptr, nullptr, nullptr, nullptr};

Motor :: Motor(uint8_t PWM, uint8_t IN1, uint8_t IN2, uint8_t ENCODER_A, uint8_t ENCODER_B){
    this -> PWM = PWM;
    this -> IN1 = IN1;
    this -> IN2 = IN2;
    this -> ENC_A = ENCODER_A;
    this -> ENC_B = ENCODER_B;
}   

void Motor :: updateEncoder() {
    // เข้ารหัสค่า
    int encoded = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
    int sum = (lastEncoded << 2) | encoded;
    int increment;

    // อัปเดตค่า encoderValue ตามทิศทางการหมุน
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
      increment = 1; 
    }
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
      increment = -1;
    }
    this -> encoderCount += increment;
    lastEncoded = encoded;

    long currentTime = micros();
    // float deltaTime = ((float) (currentTime - lastEncoded)) / 1.0e6;
    // timeInterval = currentTime - lastTimeEncoded;
    encoderVelocity = increment * 1.0e6 / ((float) (currentTime - lastTimeEncoded));
    lastTimeEncoded = currentTime;
}

void Motor::updateEncoderISR_ID0(){
    if(Encoder[0] != nullptr){
        Encoder[0] -> updateEncoder();
    }
}
void Motor::updateEncoderISR_ID1(){
    if(Encoder[1] != nullptr){
        Encoder[1] -> updateEncoder();
    }
}
void Motor::updateEncoderISR_ID2(){
    if(Encoder[2] != nullptr){
        Encoder[2] -> updateEncoder();
    }
}
void Motor::updateEncoderISR_ID3(){
    if(Encoder[3] != nullptr){
        Encoder[3] -> updateEncoder();
    }
}

long Motor :: getEncoder() { 
    return this->encoderCount;
}

double Motor :: getPosition() { 
    return (this -> encoderCount) * 2 * PI / ENCODER_PER_REVOLUTION;
}

void Motor :: info() { 
    // Serial.print("IN1 -> ");
    // Serial.print(IN1);
    // Serial.print(" | IN2 -> ");
    // Serial.print(IN2);
    // Serial.print(" | PWM -> ");
    // Serial.print(PWM);
    // Serial.print("\n EncoderA -> ");
    // Serial.print(ENC_A);
    // Serial.print(" | EncoderB -> ");
    // Serial.print(ENC_B);
    // Serial.println();
    Serial.println((long)&Encoder);
}

void Motor :: init(int motorID) {
    this -> motorID = motorID;
    pinMode(IN1, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    Encoder[motorID] = this;
    switch (motorID){
    case 0:
        attachInterrupt(digitalPinToInterrupt(ENC_A), Motor :: updateEncoderISR_ID0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoderISR_ID0, CHANGE);
        break;
    case 1:
        attachInterrupt(digitalPinToInterrupt(ENC_A), Motor :: updateEncoderISR_ID1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoderISR_ID1, CHANGE);
        break;
    case 2:
        attachInterrupt(digitalPinToInterrupt(ENC_A), Motor :: updateEncoderISR_ID2, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoderISR_ID2, CHANGE);
        break;
    case 3:
        attachInterrupt(digitalPinToInterrupt(ENC_A), Motor :: updateEncoderISR_ID3, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoderISR_ID3, CHANGE);
        break;
    default:
        break;
    }
}

void Motor :: setPulse(int pulse) { 
    updateVelocity();
    digitalWrite(IN1, pulse >= 0);
    digitalWrite(IN2, pulse <= 0);
    if (pulse == 0) {
        analogWrite(PWM, 255);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, HIGH);
    }
    else {
        analogWrite(PWM, abs(pulse));
    }
}

void Motor :: updateVelocity() { 
    encoderVelocity = ((micros() - lastTimeEncoded) > 153846.0) ? 0.0 : encoderVelocity; 
    rawVelocity = encoderVelocity * 60.0 / ENCODER_PER_REVOLUTION; 

    velocityFilted1st = velocityFilted1st * 0.854 + 0.0728 * rawVelocity + 0.0728 * previousRawVelocity;
    velocityFilted2nd = velocityFilted2nd * 0.854 + 0.0728 * velocityFilted1st + 0.0728 * previousVelocityFilted1st;
    velocity = velocity* 0.854 + 0.0728 * velocityFilted2nd + 0.0728 * previousVelocityFilted2nd;
    previousRawVelocity = rawVelocity;
    previousVelocityFilted1st = velocityFilted1st;
    previousVelocityFilted2nd = velocityFilted2nd;
    
    // Serial.print(rawVelocity);Serial.print(",");
    // Serial.print(velocityFilted1st);Serial.print(",");
    // Serial.println(velocity);
}

double Motor :: getVelocity(){
    return velocity;
}

void Motor :: setOffset(){
    this -> encoderOffset = encoderCount;
}

long Motor :: getOffset(){
    return encoderOffset;
}