#include <Wire.h>
#include "ESP32Servo.h"
#include "ESP32PWM.h"
// ========================= SERVO & MOTOR CONFIG =========================
Servo stab_servo_pitch;
Servo stab_servo_roll;
const int servo_pitch_pin = 12;
const int servo_roll_pin = 13;
Servo mot1;
Servo mot2;
const int mot1_pin = 18;
const int mot2_pin = 27;
int ESCfreq = 500;
// ========================= RECEIVER CONFIG (PPM) ========================
#define PPM_PIN 15
const int NUM_CHANNELS = 8;
volatile uint16_t ppmValues[NUM_CHANNELS] = {0};
volatile uint32_t lastTime = 0;
volatile uint8_t channelIndex = 0;
void IRAM_ATTR ppmInterrupt() 
{
    uint32_t now = micros();
    uint32_t duration = now - lastTime;
    lastTime = now;
    if (duration >= 4000) 
    {
        channelIndex = 0;
    } else if (channelIndex < NUM_CHANNELS) 
    {
        ppmValues[channelIndex++] = duration;
    }
}
// ========================= PID CONSTANTS ================================
float PAngleRoll = 2.0, IAngleRoll = 0.5, DAngleRoll = 0.007;
float PAnglePitch = PAngleRoll, IAnglePitch = IAngleRoll, DAnglePitch = DAngleRoll;
float PRateRoll = 0.625, IRateRoll = 2.1, DRateRoll = 0.0088;
float PRatePitch = PRateRoll, IRatePitch = IRateRoll, DRatePitch = DRateRoll;
float PRateYaw = 4.0, IRateYaw = 3.0, DRateYaw = 0.0;
float t = 0.004;
uint32_t LoopTimer;
// ========================= IMU VARIABLES ================================
volatile float RatePitch, RateRoll, RateYaw;
volatile float AngleRoll, AnglePitch;
volatile float AccX, AccY, AccZ;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float AccXCalibration, AccYCalibration, AccZCalibration;
// ========================= KALMAN FILTER VARIABLES ======================
float kalmanAngleRoll = 0, kalmanBiasRoll = 0;
float kalmanAnglePitch = 0, kalmanBiasPitch = 0;
float kalmanP[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
float kalmanUpdate(float newAngle, float newRate, float dt, float &kalmanAngle, float &kalmanBias, float P[2][2]) 
{
    kalmanAngle += dt * (newRate - kalmanBias);
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float y = newAngle - kalmanAngle;
    kalmanAngle += K0 * y;
    kalmanBias += K1 * y;
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;
    return kalmanAngle;
}
// ========================= SERVO PID ====================================
float servo_kp = 0.8, servo_ki = 0.2, servo_kd = 0.01;
float servo_integral_pitch = 0, servo_integral_roll = 0;
float servo_prev_error_pitch = 0, servo_prev_error_roll = 0;
float servoPID(float error, float &integral, float &prev_error) 
{
    float proportional = servo_kp * error;
    integral += servo_ki * error * t;
    integral = constrain(integral, -50, 50);
    float derivative = servo_kd * (error - prev_error) / t;
    prev_error = error;
    return proportional + integral + derivative;
}
void updateServos(float pitch_error, float roll_error) 
{
    int pitch_angle = constrain(90 + servoPID(pitch_error, servo_integral_pitch, servo_prev_error_pitch), 60, 120);
    int roll_angle = constrain(90 + servoPID(roll_error, servo_integral_roll, servo_prev_error_roll), 60, 120);
    stab_servo_pitch.write(pitch_angle);
    stab_servo_roll.write(roll_angle);
}
// ========================= IMU FUNCTIONS ================================
void gyro_signals(void) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
    RateRoll = GyroX / 65.5;
    RatePitch = GyroY / 65.5;
    RateYaw = GyroZ / 65.5;
    AccX = AccXLSB / 4096.0;
    AccY = AccYLSB / 4096.0;
    AccZ = AccZLSB / 4096.0;
    AngleRoll = atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 57.29;
    AnglePitch = -atan(AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 57.29;
}
// ========================= SETUP ================================
void setup() 
{
    Serial.begin(115200);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    stab_servo_pitch.setPeriodHertz(50);
    stab_servo_roll.setPeriodHertz(50);
    stab_servo_pitch.attach(servo_pitch_pin, 500, 2400);
    stab_servo_roll.attach(servo_roll_pin, 500, 2400);
    mot1.setPeriodHertz(ESCfreq);
    mot2.setPeriodHertz(ESCfreq);
    mot1.attach(mot1_pin, 1000, 2000);
    mot2.attach(mot2_pin, 1000, 2000);
    // This sends the initial 1000us signal required for ESCs to be ready to arm
    mot1.writeMicroseconds(1000);
    mot2.writeMicroseconds(1000);
    pinMode(PPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, RISING);
    #define MPU_ADDR 0x68
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    RateCalibrationRoll = 0.27; RateCalibrationPitch = -0.85; RateCalibrationYaw = -2.09;
    AccXCalibration = 0.03; AccYCalibration = 0.01; AccZCalibration = -0.07;
    LoopTimer = micros();
    Wire.setClock(400000);
    delay(100);
}
// ========================= LOOP ================================
// Global variables for arming status
bool armed = false;
int armingState = 0; // 0: initial, 1: throttle low, 2: Armed
void loop() 
{
    // 1. Read IMU and PPM signals (all at the beginning)
    gyro_signals();
    float InputThrottle = ppmValues[2];
    float InputYaw = ppmValues[3];
    float InputRoll = ppmValues[0];
    float InputPitch = ppmValues[1];
    // Declare motor output variables so they are in scope for the debug print
    int mot1_output = 1000;
    int mot2_output = 1000;
    // 2. Apply IMU calibration values
    RateRoll -= RateCalibrationRoll; RatePitch -= RateCalibrationPitch; RateYaw -= RateCalibrationYaw;
    AccX -= AccXCalibration; AccY -= AccYCalibration; AccZ -= AccZCalibration;
    // 3. Run Kalman filter to get stable angles
    kalmanAngleRoll = kalmanUpdate(AngleRoll, RateRoll, t, kalmanAngleRoll, kalmanBiasRoll, kalmanP);
    kalmanAnglePitch = kalmanUpdate(AnglePitch, RatePitch, t, kalmanAnglePitch, kalmanBiasPitch, kalmanP);
    kalmanAngleRoll = constrain(kalmanAngleRoll, -20, 20);
    kalmanAnglePitch = constrain(kalmanAnglePitch, -20, 20);
    // 4. Calculate desired angles and rates from RC input
    float DesiredAngleRoll = 0.1 * (ppmValues[0] - 1500);
    float DesiredAnglePitch = 0.1 * (ppmValues[1] - 1500);
    float DesiredRateYaw = 0.15 * (ppmValues[3] - 1500);
    // 5. Arming Logic (State Machine)
    switch (armingState) 
    {
        case 0: // Waiting for throttle-low stick
            if (InputThrottle < 1120) 
            {
                armingState = 1;
                Serial.println("Arming sequence started. Move yaw stick to the far right to arm.");
            } 
            else 
            {
                Serial.println("Waiting for throttle to be low to begin arming sequence.");
            }
            break;
        case 1: // Waiting for yaw-right stick
            if (InputThrottle < 1120 && InputYaw > 1800) 
            {
                armingState = 2;
                armed = true;
                Serial.println("ESCs Armed!");
            } else if (InputThrottle > 1120) 
            {
                 armingState = 0; // Reset state if throttle is raised
                 Serial.println("Arming sequence reset. Throttle must be low to start.");
            } else 
            {
                Serial.println("Waiting for yaw stick to be at the far right.");
            }
            break;
        case 2: // Armed state - main control loop
            // The throttle value is mapped from the range of your transmitter
            // (1100-2000) to the full range of the ESC (1000-2000).
            mot1_output = map(InputThrottle, 1609, 2000, 1000, 2000);
            mot2_output = map(InputThrottle, 1609, 2000, 1000, 2000);
            // Disarm condition: Throttle low and yaw far left
            if (InputThrottle < 1120 && InputYaw < 1200) 
            {
                armingState = 0;
                armed = false;
                mot1.writeMicroseconds(1000);
                mot2.writeMicroseconds(1000);
                Serial.println("Disarmed");
            }
            // Direct mapping of RC input to servo output
            int pitch_angle = map(InputPitch, 1125, 2000, 30,60);
            int roll_angle = map(InputRoll, 1009, 2000, 30, 60);
            stab_servo_pitch.write(pitch_angle);
            stab_servo_roll.write(roll_angle);
            break;
    }
    // 6. Failsafe check: if PPM value is below 900
    if (InputThrottle < 900) 
    {
        armingState = 0;
        armed = false;
        mot1.writeMicroseconds(1000);
        mot2.writeMicroseconds(1000);
        Serial.println("Failsafe activated: Disarming due to low throttle/signal loss.");
    }
    // 7. Write to motors (moved outside of the arming logic)
    mot1.writeMicroseconds(mot1_output);
    mot2.writeMicroseconds(mot2_output);
    // 8. Debug printing
    Serial.print("CH1:"); Serial.print(ppmValues[0]);
    Serial.print(" CH2:"); Serial.print(ppmValues[1]);
    Serial.print(" CH3:"); Serial.print(ppmValues[2]);
    Serial.print(" CH4:"); Serial.print(ppmValues[3]);
    Serial.print(" | Roll:"); Serial.print(kalmanAngleRoll);
    Serial.print(" Pitch:"); Serial.println(kalmanAnglePitch);
    Serial.print(" | Arming State: "); Serial.println(armingState);
    Serial.print(" | Motor Output: "); Serial.println(mot1_output);
    // 9. Loop timer to maintain a fixed loop time (4000 us)
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
}
