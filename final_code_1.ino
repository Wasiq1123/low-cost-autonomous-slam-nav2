#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <cmath>
#include <algorithm>
#include <cstdio>

constexpr const char* WIFI_SSID = "name";  // Replace with your Wi-Fi network name
constexpr const char* WIFI_PASSWORD = "password"; // Replace with your Wi-Fi password
constexpr uint16_t ROS_SERVER_PORT = 80;

WiFiServer rosServer(ROS_SERVER_PORT);
WiFiClient rosClient;
char rosBuffer[128];
uint8_t rosBufferIndex = 0;

constexpr int LEFT_MOTOR_REN  = 26;
constexpr int LEFT_MOTOR_LEN  = 25;
constexpr int LEFT_MOTOR_RPWM = 33;
constexpr int LEFT_MOTOR_LPWM = 32;

constexpr int RIGHT_MOTOR_REN  = 13;
constexpr int RIGHT_MOTOR_LEN  = 12;
constexpr int RIGHT_MOTOR_RPWM = 14;
constexpr int RIGHT_MOTOR_LPWM = 27;

constexpr int LEFT_ENCODER_PIN  = 4;
constexpr int RIGHT_ENCODER_PIN = 15;
constexpr int I2C_SDA_PIN = 22;
constexpr int I2C_SCL_PIN = 23;

constexpr float WHEEL_RADIUS = 0.033f;
constexpr float SLOTS_PER_REV = 20.0f;
constexpr float WHEEL_CIRCUMFERENCE = 2.0f * M_PI * WHEEL_RADIUS;
constexpr float SLOTS_PER_METER = SLOTS_PER_REV / WHEEL_CIRCUMFERENCE;
constexpr unsigned long ENCODER_DEBOUNCE_US = 300;

MPU6050 mpu;
float axOffset = 0.0f, ayOffset = 0.0f, azOffset = 0.0f;
float gxOffset = 0.0f, gyOffset = 0.0f, gzOffset = 0.0f;
float yaw = 0.0f;
unsigned long prevTime = 0;

volatile long leftCount = 0;
volatile long rightCount = 0;
volatile unsigned long leftLastPulse = 0;
volatile unsigned long rightLastPulse = 0;
volatile int left_motor_dir = 1;
volatile int right_motor_dir = 1;

void applyMotorPWM(int pwm, int r_pwm_pin, int l_pwm_pin, volatile int &motor_dir) {
    if (pwm > 0) {
        analogWrite(r_pwm_pin, pwm);
        analogWrite(l_pwm_pin, 0);
        motor_dir = 1;
    } else if (pwm < 0) {
        analogWrite(r_pwm_pin, 0);
        analogWrite(l_pwm_pin, std::abs(pwm));
        motor_dir = -1;
    } else {
        analogWrite(r_pwm_pin, 0);
        analogWrite(l_pwm_pin, 0);
    }
}

void stopMotors() {
    applyMotorPWM(0, LEFT_MOTOR_RPWM, LEFT_MOTOR_LPWM, left_motor_dir);
    applyMotorPWM(0, RIGHT_MOTOR_RPWM, RIGHT_MOTOR_LPWM, right_motor_dir);
}

void resetEncoders() {
    noInterrupts();
    leftCount = 0;
    rightCount = 0;
    interrupts();
}

void IRAM_ATTR rightEncoderISR() {
    unsigned long currentTime = micros();
    if (currentTime - rightLastPulse > ENCODER_DEBOUNCE_US) {
        rightCount += right_motor_dir;
        rightLastPulse = currentTime;
    }
}

void IRAM_ATTR leftEncoderISR() {
    unsigned long currentTime = micros();
    if (currentTime - leftLastPulse > ENCODER_DEBOUNCE_US) {
        leftCount += left_motor_dir;
        leftLastPulse = currentTime;
    }
}

void calibrateIMU() {
    long long axSum = 0, aySum = 0, azSum = 0;
    long long gxSum = 0, gySum = 0, gzSum = 0;
    constexpr int SAMPLES = 2000;
    Serial.println("Calibrating IMU (keep still)...");
    for (int i = 0; i < SAMPLES; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        axSum += ax; aySum += ay; azSum += az;
        gxSum += gx; gySum += gy; gzSum += gz;
        if (i % 200 == 0) Serial.print(".");
        delay(5);
    }
    Serial.println("\nDone.");
    axOffset = (float)axSum / SAMPLES;
    ayOffset = (float)aySum / SAMPLES;
    azOffset = ((float)azSum / SAMPLES) - 16384.0f;
    gxOffset = (float)gxSum / SAMPLES;
    gyOffset = (float)gySum / SAMPLES;
    gzOffset = (float)gzSum / SAMPLES;
}

void processCommand(const char* line) {
    int left_pwm = 0, right_pwm = 0;
    if (sscanf(line, "PL:%d,PR:%d", &left_pwm, &right_pwm) == 2) {
        left_pwm = std::clamp(left_pwm, -255, 255);
        right_pwm = std::clamp(right_pwm, -255, 255);
        Serial.printf("CMD: L=%d, R=%d\n", left_pwm, right_pwm);
        applyMotorPWM(left_pwm, LEFT_MOTOR_RPWM, LEFT_MOTOR_LPWM, left_motor_dir);
        applyMotorPWM(right_pwm, RIGHT_MOTOR_RPWM, RIGHT_MOTOR_LPWM, right_motor_dir);
        return;
    }
    if (line[0] == 'S' || line[0] == 's') { stopMotors(); return; }
    if (line[0] == 'Z' || line[0] == 'z') { resetEncoders(); return; }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESP32 CONTROLLER v3 (50ms vel + EMA) ===");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);
    mpu.initialize();
    if (!mpu.testConnection()) { Serial.println("FATAL: MPU6050!"); while(1) delay(1000); }
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    delay(2000);
    calibrateIMU();
    yaw = 0.0f;
    prevTime = micros();

    pinMode(LEFT_MOTOR_REN, OUTPUT); digitalWrite(LEFT_MOTOR_REN, HIGH);
    pinMode(LEFT_MOTOR_LEN, OUTPUT); digitalWrite(LEFT_MOTOR_LEN, HIGH);
    pinMode(RIGHT_MOTOR_REN, OUTPUT); digitalWrite(RIGHT_MOTOR_REN, HIGH);
    pinMode(RIGHT_MOTOR_LEN, OUTPUT); digitalWrite(RIGHT_MOTOR_LEN, HIGH);
    pinMode(LEFT_MOTOR_RPWM, OUTPUT);
    pinMode(LEFT_MOTOR_LPWM, OUTPUT);
    pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
    pinMode(RIGHT_MOTOR_LPWM, OUTPUT);
    stopMotors();

    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, FALLING);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
    rosServer.begin();
    Serial.println("Server ready.");
}

void loop() {
    WiFiClient newClient = rosServer.available();
    if (newClient) {
        if (rosClient && rosClient.connected()) rosClient.stop();
        rosClient = newClient;
        Serial.println("ROS2 connected!");
        rosBufferIndex = 0;
    }

    if (rosClient && rosClient.connected()) {
        while (rosClient.available()) {
            char c = rosClient.read();
            if (c == '\n') {
                rosBuffer[rosBufferIndex] = '\0';
                processCommand(rosBuffer);
                rosBufferIndex = 0;
            } else if (c != '\r') {
                if (rosBufferIndex < sizeof(rosBuffer) - 1) rosBuffer[rosBufferIndex++] = c;
            }
        }
    } else if (rosClient) {
        rosClient.stop();
    }

    unsigned long now = micros();
    float dt = (now - prevTime) / 1000000.0f;
    prevTime = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ax_mps2 = ((ax - axOffset) / 16384.0f) * 9.80665f;
    float ay_mps2 = ((ay - ayOffset) / 16384.0f) * 9.80665f;
    float az_mps2 = ((az - azOffset) / 16384.0f) * 9.80665f;

    float gz_rad_s = (((gz - gzOffset) / 131.0f) * 0.01745329252f);
    if (std::abs(gz_rad_s) < 0.01f) gz_rad_s = 0.0f;
    yaw += gz_rad_s * dt;
    if (yaw > M_PI) yaw -= 2.0f * M_PI;
    else if (yaw < -M_PI) yaw += 2.0f * M_PI;

    // ✅ VELOCITY: 50ms window + EMA filter (fixes oscillation)
    noInterrupts();
    long currentLeft = leftCount;
    long currentRight = rightCount;
    interrupts();

    static float prevLeftSlots = 0.0f, prevRightSlots = 0.0f;
    static unsigned long prevVelTime = micros();
    static float filteredLV = 0.0f, filteredRV = 0.0f;

    float dtVel = (now - prevVelTime) / 1000000.0f;
    float leftVelocity = filteredLV;
    float rightVelocity = filteredRV;

    if (dtVel >= 0.05f) {
        prevVelTime = now;
        float lSlots = currentLeft / 2.0f;
        float rSlots = currentRight / 2.0f;
        float rawLV = ((lSlots - prevLeftSlots) / SLOTS_PER_METER) / dtVel;
        float rawRV = ((rSlots - prevRightSlots) / SLOTS_PER_METER) / dtVel;
        prevLeftSlots = lSlots;
        prevRightSlots = rSlots;

        constexpr float ALPHA = 0.3f;
        filteredLV = ALPHA * rawLV + (1.0f - ALPHA) * filteredLV;
        filteredRV = ALPHA * rawRV + (1.0f - ALPHA) * filteredRV;
        leftVelocity = filteredLV;
        rightVelocity = filteredRV;
    }

    if (rosClient && rosClient.connected()) {
        char out[160];
        snprintf(out, sizeof(out),
            "L:%ld,R:%ld,Y:%.4f,VL:%.3f,VR:%.3f,AX:%.3f,AY:%.3f,AZ:%.3f,GX:%.4f,GY:%.4f,GZ:%.4f\n",
            (long)(currentLeft/2), (long)(currentRight/2), yaw,
            leftVelocity, rightVelocity,
            ax_mps2, ay_mps2, az_mps2, 0.0f, 0.0f, gz_rad_s);
        rosClient.print(out);
    }

    delay(10);
}
