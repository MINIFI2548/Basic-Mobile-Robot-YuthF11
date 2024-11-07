#include <Arduino.h>

#ifdef NUM_MOTORS
#else 
    #define NUM_MOTORS 4 
#endif

#define RADIUS_OF_WHEELS 39 //mm 
#define ENCODER_PER_REVOLUTION 3900 // tick per revolurion 

class Motor{
    private :
        volatile long lastEncoded;
        volatile long encoderCount;
        volatile long lastTimeEncoded; // last time encoder ticker 
        volatile long timeInterval; // time between encoder tick
        volatile float encoderVelocity;
        float velocity, rawVelocity, velocityFilted1st,velocityFilted2nd,previousVelocityFilted2nd, previousRawVelocity, previousVelocityFilted1st;
        // velocity in this class mean angular velocity(RPM)
        long lastTime;
        long encoderOffset;

        int motorID = 0;

        static Motor *Encoder[NUM_MOTORS];
        static void updateEncoderISR_ID0();
        static void updateEncoderISR_ID1();
        static void updateEncoderISR_ID2();
        static void updateEncoderISR_ID3();

    public :
        uint8_t PWM, IN1, IN2, ENC_A, ENC_B;
        Motor(uint8_t PWM, uint8_t IN1, uint8_t IN2, uint8_t ENCODER_A, uint8_t ENCODER_B);

        long getEncoder();
        double getPosition();
        double getVelocity();
        long getOffset();

        void updateVelocity();
        void info();
        void init(int motorID);
        void updateEncoder();
        void setPulse(int pluse);
        void setOffset(); 
};