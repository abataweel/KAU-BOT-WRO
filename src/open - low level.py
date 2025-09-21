```cpp
#include <Wire.h>
#include "DFRobot_BNO055.h"
#include <Servo.h>

/* ================= Configuration ================= */
#define BUZZER_PIN A0
#define START_BUTTON A1
#define START_ACTIVE_LOW 1
#define BUZZER_ENABLED 1

// BTS7960 Motor Driver Pins
#define MOTOR_RPWM 5
#define MOTOR_LPWM 6
#define MOTOR_R_EN 7
#define MOTOR_L_EN 8

// Servo
#define SERVO_PIN 9
int SERVO_CENTER = 90;
#define SERVO_MIN 65
#define SERVO_MAX 120
float STEER_SPAN_LEFT = 30.0f;
float STEER_SPAN_RIGHT = 30.0f;
float STEER_TRIM = 0.0f;

// Ultrasonic Sensors
#define US_RIGHT_PIN 2
#define US_LEFT_PIN 3
#define US_FRONT_PIN 4
#define US_MAX_RANGE_CM 300
int16_t ULTRASONIC_TEMP_C = 20;

// IMU
#define IMU_ADDRESS 0x28
#define IMU_STALE_TIMEOUT_MS 400
#define IMU_MAX_BAD_READS 3

// Control Parameters
#define SPEED_MIN 0
#define SPEED_MAX 255
#define TURN_PWM_DEFAULT 120
#define TURN_TOLERANCE_DEG 2.0f
#define TURN_HOLD_DURATION_MS 100
#define STEER_FILTER_ALPHA 0.18f
#define SPEED_FILTER_ALPHA 0.35f
#define PID_D_FILTER_ALPHA 0.35f

/* ================= Types and State ================= */
enum RobotState { IDLE = 0, DRIVING = 1, TURNING = 2 };
RobotState currentState = IDLE;
bool isArmed = false;

struct MotorState {
  int speed = 0;
  int direction = 0; // +1 forward, -1 reverse, 0 stop
  float steerNorm = 0.0f;
};

struct PIDController {
  float Kp = 3.0f;
  float Ki = 0.0f;
  float Kd = 0.26f;
  float integral = 0.0f;
  float prevError = 0.0f;
  float filteredD = 0.0f;
};

/* ================= Global Objects and Variables ================= */
Servo steeringServo;
DFRobot_BNO055_IIC imu(&Wire, IMU_ADDRESS);
MotorState motor;
PIDController pid;
float currentYaw = 0.0f;
float targetYaw = 0.0f;
int turnPwmOverride = -1;
long distances[3] = {-1, -1, -1}; // Front, Left, Right
unsigned long lastImuUpdate = 0;
int imuErrorCount = 0;
bool turnCompleted = false;
unsigned long turnStableStart = 0;
unsigned long lastPidUpdate = 0;
float filteredSteer = 0.0f;
float filteredSpeed = 0.0f;

/* ================= Helper Functions ================= */
float calculateSoundSpeed(int tempC) {
  return (331.5 + 0.6 * (float)tempC) * 100 / 1000000.0; // cm/us
}

void updateSteeringSpans() {
  STEER_SPAN_LEFT = (float)(SERVO_CENTER - SERVO_MIN);
  STEER_SPAN_RIGHT = (float)(SERVO_MAX - SERVO_CENTER);
}

float wrapAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

float yawError(float target, float current) {
  return wrapAngle(target - current);
}

/* ================= Buzzer Functions ================= */
void playBeep(uint16_t duration = 80) {
#if BUZZER_ENABLED
  tone(BUZZER_PIN, 3000, duration);
#endif
}

void rapidChirp() {
#if BUZZER_ENABLED
  static unsigned long lastBuzz = 0;
  if (currentState != TURNING) return;
  unsigned long now = millis();
  if (now - lastBuzz >= 80) {
    lastBuzz = now;
    tone(BUZZER_PIN, 3500, 40);
  }
#endif
}

/* ================= Motor Control ================= */
void initializeMotor() {
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);
  digitalWrite(MOTOR_R_EN, HIGH);
  digitalWrite(MOTOR_L_EN, HIGH);
  stopMotor();
}

void stopMotor() {
  analogWrite(MOTOR_RPWM, 0);
  analogWrite(MOTOR_LPWM, 0);
}

void driveMotor(int pwm, int direction) {
  pwm = constrain(pwm, SPEED_MIN, SPEED_MAX);
  if (direction > 0) {
    analogWrite(MOTOR_RPWM, pwm);
    analogWrite(MOTOR_LPWM, 0);
  } else if (direction < 0) {
    analogWrite(MOTOR_RPWM, 0);
    analogWrite(MOTOR_LPWM, pwm);
  } else {
    stopMotor();
  }
}

/* ================= Servo Control ================= */
void setSteeringAngle(int angle) {
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  steeringServo.write(angle);
}

void setSteeringNormalized(float norm) {
  norm = constrain(norm + STEER_TRIM, -1.0f, 1.0f);
  if (fabs(norm) < 0.02f) norm = 0.0f;
  float angle = (norm >= 0.0f) ? (SERVO_CENTER + norm * STEER_SPAN_RIGHT)
                              : (SERVO_CENTER + norm * STEER_SPAN_LEFT);
  setSteeringAngle((int)angle);
}

/* ================= IMU Functions ================= */
bool initializeIMU() {
  imu.setOprMode(DFRobot_BNO055_IIC::eOprModeConfig);
  delay(25);
  if (imu.begin() != DFRobot_BNO055_IIC::eStatusOK) return false;
  imu.setOprMode(DFRobot_BNO055_IIC::eOprModeNdof);
  delay(30);
  lastImuUpdate = millis();
  imuErrorCount = 0;
  return true;
}

bool resetIMU() {
  bool success = initializeIMU();
  Serial.println(success ? "INFO,IMU_RESET_OK" : "WARN,IMU_RESET_FAIL");
  return success;
}

void updateYaw() {
  DFRobot_BNO055_IIC::sEulAnalog_t euler = imu.getEul();
  unsigned long now = millis();
  bool isValid = (imu.lastOperateStatus == DFRobot_BNO055_IIC::eStatusOK);
  bool isZeroFrame = (fabs(euler.head) < 0.001f && fabs(euler.roll) < 0.001f && fabs(euler.pitch) < 0.001f);

  if (isValid && !isZeroFrame && !isnan(euler.head)) {
    currentYaw = wrapAngle(euler.head);
    lastImuUpdate = now;
    imuErrorCount = 0;
  } else {
    imuErrorCount++;
  }

  if (imuErrorCount >= IMU_MAX_BAD_READS || (now - lastImuUpdate) > IMU_STALE_TIMEOUT_MS) {
    resetIMU();
  }
}

/* ================= Ultrasonic Functions ================= */
unsigned long calculateTimeout(int rangeCm) {
  return (unsigned long)((2.0f * rangeCm) / calculateSoundSpeed(ULTRASONIC_TEMP_C) * 1.2f);
}

long readUltrasonic(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);

  unsigned long timeout = calculateTimeout(US_MAX_RANGE_CM);
  unsigned long pulseWidth = pulseIn(pin, HIGH, timeout);
  if (pulseWidth == 0) return -1;

  long distance = (long)(pulseWidth * calculateSoundSpeed(ULTRASONIC_TEMP_C) / 2.0f);
  return constrain(distance, 0, US_MAX_RANGE_CM);
}

void updateUltrasonics() {
  distances[0] = readUltrasonic(US_FRONT_PIN);
  distances[1] = readUltrasonic(US_LEFT_PIN);
  distances[2] = readUltrasonic(US_RIGHT_PIN);
}

/* ================= Turn Controller ================= */
void executeTurn() {
  int pwm = (turnPwmOverride >= 0) ? turnPwmOverride : TURN_PWM_DEFAULT;
  pwm = constrain(pwm, SPEED_MIN, SPEED_MAX);

  float error = yawError(targetYaw, currentYaw);
  if (fabs(error) < 1.0f) error = 0.0f;

  unsigned long now = millis();
  float dt = (lastPidUpdate == 0) ? 0.02f : (now - lastPidUpdate) / 1000.0f;
  lastPidUpdate = now;

  pid.integral += error * dt;
  float derivative = (error - pid.prevError) / max(dt, 1e-3f);
  pid.filteredD = PID_D_FILTER_ALPHA * derivative + (1.0f - PID_D_FILTER_ALPHA) * pid.filteredD;
  pid.prevError = error;

  float steerCommand = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * pid.filteredD;
  float normalizedSteer = constrain(steerCommand / 45.0f, -1.2f, 1.2f);

  setSteeringNormalized(-normalizedSteer);
  driveMotor(pwm, 1);

  if (fabs(error) <= TURN_TOLERANCE_DEG) {
    if (!turnCompleted) {
      turnStableStart = now;
      turnCompleted = true;
    } else if (now - turnStableStart >= TURN_HOLD_DURATION_MS) {
      stopMotor();
      setSteeringAngle(SERVO_CENTER);
      currentState = IDLE;
      pid.integral = 0;
      pid.prevError = 0;
      pid.filteredD = 0;
      lastPidUpdate = 0;
      turnCompleted = false;
      turnPwmOverride = -1;
#if BUZZER_ENABLED
      noTone(BUZZER_PIN);
#endif
      playBeep(60);
    }
  } else {
    turnCompleted = false;
  }
}

/* ================= Serial Command Handler ================= */
void processCommand(String command) {
  command.trim();
  if (command.equalsIgnoreCase("PING")) {
    Serial.println("PONG");
    return;
  }

  if (command.equalsIgnoreCase("START") || command.equalsIgnoreCase("S")) {
    setArmed(true);
    return;
  }

  if (!isArmed) return;

  if (command.startsWith("CENTER")) {
    int comma1 = command.indexOf(',');
    int comma2 = command.indexOf(',', comma1 + 1);
    if (comma1 > 0 && comma2 > comma1) {
      motor.steerNorm = constrain(command.substring(comma1 + 1, comma2).toFloat(), -1.0f, 1.0f);
      motor.speed = constrain(command.substring(comma2 + 1).toInt(), SPEED_MIN, SPEED_MAX);
      motor.direction = (motor.speed > 0) ? 1 : 0;
      if (currentState != TURNING) currentState = DRIVING;
#if BUZZER_ENABLED
      noTone(BUZZER_PIN);
#endif
    }
  } else if (command.startsWith("BACK")) {
    int comma = command.indexOf(',');
    if (comma > 0) {
      motor.speed = constrain(command.substring(comma + 1).toInt(), SPEED_MIN, SPEED_MAX);
      motor.direction = (motor.speed > 0) ? -1 : 0;
      if (currentState != TURNING) currentState = DRIVING;
#if BUZZER_ENABLED
      noTone(BUZZER_PIN);
#endif
    }
  } else if (command.startsWith("TURN_ABS")) {
    int comma1 = command.indexOf(',');
    if (comma1 > 0) {
      int comma2 = command.indexOf(',', comma1 + 1);
      targetYaw = wrapAngle(command.substring(comma1 + 1, comma2 > 0 ? comma2 : command.length()).toFloat());
      turnPwmOverride = (comma2 > 0) ? command.substring(comma2 + 1).toInt() : -1;
      currentState = TURNING;
      pid.integral = 0;
      pid.prevError = 0;
      pid.filteredD = 0;
      lastPidUpdate = 0;
      turnCompleted = false;
      playBeep(120);
    }
  } else if (command.startsWith("TRIM_NORM")) {
    int comma = command.indexOf(',');
    if (comma > 0) {
      STEER_TRIM = constrain(command.substring(comma + 1).toFloat(), -0.3f, 0.3f);
      playBeep(40);
    }
  } else if (command.startsWith("SET_CENTER")) {
    int comma = command.indexOf(',');
    if (comma > 0) {
      SERVO_CENTER = constrain(command.substring(comma + 1).toInt(), SERVO_MIN, SERVO_MAX);
      updateSteeringSpans();
      setSteeringAngle(SERVO_CENTER);
      playBeep(80);
    }
  } else if (command.startsWith("SET_TURN_PWM")) {
    int comma = command.indexOf(',');
    if (comma > 0) {
      TURN_PWM_DEFAULT = constrain(command.substring(comma + 1).toInt(), SPEED_MIN, SPEED_MAX);
      playBeep(60);
    }
  } else if (command.equalsIgnoreCase("CLEAR_TURN_PWM")) {
    turnPwmOverride = -1;
    playBeep(40);
  } else if (command.equalsIgnoreCase("STOP")) {
    currentState = IDLE;
    motor.speed = 0;
    motor.direction = 0;
    stopMotor();
    setSteeringNormalized(0.0f);
#if BUZZER_ENABLED
    noTone(BUZZER_PIN);
#endif
  }
}

/* ================= Telemetry ================= */
void sendTelemetry() {
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry < 25) return;
  lastTelemetry = millis();
  updateUltrasonics();
  Serial.print("TLM,yaw=");
  Serial.print(currentYaw, 1);
  Serial.print(",state=");
  Serial.print((int)currentState);
  Serial.print(",steer=");
  Serial.print(motor.steerNorm, 2);
  Serial.print(",speed=");
  Serial.print(motor.speed);
  Serial.print(",dF=");
  Serial.print(distances[0]);
  Serial.print(",dL=");
  Serial.print(distances[1]);
  Serial.print(",dR=");
  Serial.println(distances[2]);
}

/* ================= Start Button Handling ================= */
bool isButtonPressed() {
  if (!digitalRead(START_BUTTON) == START_ACTIVE_LOW) return false;
  delay(20);
  return digitalRead(START_BUTTON) == START_ACTIVE_LOW;
}

void setArmed(bool state) {
  if (state && !isArmed) {
    isArmed = true;
    playBeep(100);
    Serial.println("ARMED");
  } else if (!state && isArmed) {
    isArmed = false;
  }
}

void waitForStartSignal() {
  stopMotor();
  setSteeringAngle(SERVO_CENTER);
  String buffer = "";
  while (!isArmed) {
    if (isButtonPressed()) {
      setArmed(true);
      while (digitalRead(START_BUTTON) == START_ACTIVE_LOW) delay(5);
      break;
    }
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (buffer.equalsIgnoreCase("S") || buffer.equalsIgnoreCase("START")) {
          setArmed(true);
          break;
        }
        buffer = "";
      } else if (buffer.length() < 32) {
        buffer += c;
      }
    }
    delay(2);
  }
}

/* ================= Setup and Loop ================= */
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(START_BUTTON, INPUT_PULLUP);
  initializeMotor();
  steeringServo.attach(SERVO_PIN);
  updateSteeringSpans();
  setSteeringAngle(SERVO_CENTER);
  Wire.begin();
  Wire.setClock(100000);
  initializeIMU();
  Serial.println("READY");
  waitForStartSignal();
  currentState = IDLE;
}

void loop() {
  rapidChirp();
  updateYaw();

  // Handle Serial Input
  static String serialBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (serialBuffer.length() < 128) {
      serialBuffer += c;
    }
  }

  // Check Button for Arming
  if (!isArmed && isButtonPressed()) {
    setArmed(true);
    while (digitalRead(START_BUTTON) == START_ACTIVE_LOW) delay(5);
  }

  // Safety Stop for IMU Failure
  if (currentState == TURNING && (millis() - lastImuUpdate) > IMU_STALE_TIMEOUT_MS) {
    stopMotor();
    setSteeringAngle(SERVO_CENTER);
    currentState = IDLE;
#if BUZZER_ENABLED
    noTone(BUZZER_PIN);
#endif
    Serial.println("WARN,IMU_STALE_STOP");
  }

  // State Machine
  switch (currentState) {
    case IDLE:
      stopMotor();
      setSteeringNormalized(0.0f);
      break;
    case DRIVING:
      filteredSteer += STEER_FILTER_ALPHA * (motor.steerNorm - filteredSteer);
      filteredSpeed += SPEED_FILTER_ALPHA * ((float)motor.speed - filteredSpeed);
      setSteeringNormalized(filteredSteer);
      driveMotor((int)filteredSpeed, motor.direction);
      break;
    case TURNING:
      executeTurn();
      break;
  }

  sendTelemetry();
}
``