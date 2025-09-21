#include <Wire.h>
#include <DFRobot_BNO055.h>
#include <Servo.h>

/* ========== Hardware Configuration ========== */
#define BUZZER_PIN A0
#define START_BTN  A1
#define RPWM 5
#define LPWM 6
#define R_EN 7
#define L_EN 8
#define SERVO_PIN 9
#define US_F_PIN 4  // Front ultrasonic
#define US_L_PIN 3  // Left ultrasonic
#define US_R_PIN 2  // Right ultrasonic
#define US_B_PIN 10 // Rear ultrasonic

/* ========== Constants ========== */
#define BUZZER_ENABLED 1
#define SERVO_CENTER 90
#define SERVO_MIN 65
#define SERVO_MAX 120
#define SERVO_DIR -1  // -1 for reversed servo
#define SPEED_MIN 0
#define SPEED_MAX 255
#define US_TIMEOUT 120000UL  // µs
#define US_FAR_CM 999
#define IMU_STALE_MS 400
#define IMU_BAD_LIMIT 3
#define TURN_TOL_DEG 2.5
#define TURN_MAX_MS 2500
#define OBS_FRONT_CM 30
#define OBS_SIDE_CM 20

/* ========== Classes and Global Objects ========== */
class RobotControl {
private:
  DFRobot_BNO055_IIC bno;
  Servo servo;
  float yaw_deg = 0.0f;
  float target_yaw = 0.0f;
  float steer_norm = 0.0f;
  int speed = 0;
  int direction = 0;  // +1 forward, -1 reverse, 0 stop
  float steer_filt = 0.0f;
  float speed_filt = 0.0f;
  float pid_i = 0.0f, prev_err = 0.0f, d_filt = 0.0f;
  unsigned long last_pid_ms = 0;
  unsigned long turn_start_ms = 0;
  unsigned long last_imu_ok_ms = 0;
  int imu_bad_count = 0;
  long dist[4] = {-1, -1, -1, -1};  // Front, Left, Right, Rear
  bool armed = false;
  bool buzzer_rapid = false;
  unsigned long buzz_last_ms = 0;
  float span_left = SERVO_CENTER - SERVO_MIN;
  float span_right = SERVO_MAX - SERVO_CENTER;
  float steer_trim = 0.0f;
  int turn_pwm = 22;
  int turn_pwm_override = -1;
  int turn_dir = 1;

public:
  enum State { IDLE, CENTERING, TURNING };
  State state = IDLE;

  RobotControl() : bno(&Wire, 0x28) {}

  void begin() {
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(START_BTN, INPUT_PULLUP);
    pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
    pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
    digitalWrite(R_EN, HIGH); digitalWrite(L_EN, HIGH);
    servo.attach(SERVO_PIN);
    steer_to(SERVO_CENTER);
    Wire.begin();
    Wire.setClock(100000);
    init_imu();
    Serial.begin(115200);
    Serial.println("READY");
  }

  void beep(uint16_t ms = 80) {
#if BUZZER_ENABLED
    tone(BUZZER_PIN, 3000, ms);
#endif
  }

  void chirp_rapid() {
#if BUZZER_ENABLED
    if (!buzzer_rapid) return;
    unsigned long now = millis();
    if (now - buzz_last_ms >= 80) {
      buzz_last_ms = now;
      tone(BUZZER_PIN, 3500, 40);
    }
#endif
  }

  void init_imu() {
    bno.setOprMode(DFRobot_BNO055_IIC::eOprModeConfig);
    delay(25);
    if (bno.begin() == DFRobot_BNO055_IIC::eStatusOK) {
      bno.setOprMode(DFRobot_BNO055_IIC::eOprModeNdof);
      delay(30);
      last_imu_ok_ms = millis();
      imu_bad_count = 0;
      Serial.println("INFO,IMU_OK");
    } else {
      Serial.println("WARN,IMU_FAIL");
    }
  }

  void read_imu() {
    DFRobot_BNO055_IIC::sEulAnalog_t e = bno.getEul();
    unsigned long now = millis();
    bool ok = (bno.lastOperateStatus == DFRobot_BNO055_IIC::eStatusOK);
    bool zero_frame = (fabs(e.head) < 0.001f && fabs(e.roll) < 0.001f && fabs(e.pitch) < 0.001f);
    if (ok && !zero_frame && !isnan(e.head)) {
      yaw_deg = wrap180(e.head);
      last_imu_ok_ms = now;
      imu_bad_count = 0;
    } else {
      imu_bad_count++;
      if (imu_bad_count >= IMU_BAD_LIMIT || (now - last_imu_ok_ms) > IMU_STALE_MS) {
        init_imu();
      }
    }
  }

  long read_ultrasonic(int pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); delayMicroseconds(2);
    digitalWrite(pin, HIGH); delayMicroseconds(10);
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);
    unsigned long pw = pulseIn(pin, HIGH, US_TIMEOUT);
    if (pw == 0) return US_FAR_CM;
    return (long)(pw * (331.5 + 0.6 * 20.0) * 100 / 2000000.0);
  }

  void read_sensors() {
    static uint8_t phase = 0;
    switch (phase) {
      case 0: {
        long a = read_ultrasonic(US_F_PIN);
        long b = read_ultrasonic(US_F_PIN);
        dist[0] = (a < 0) ? b : (b < 0) ? a : min(a, b);
        break;
      }
      case 1: dist[1] = read_ultrasonic(US_L_PIN); break;
      case 2: dist[2] = read_ultrasonic(US_R_PIN); break;
      case 3: dist[3] = read_ultrasonic(US_B_PIN); break;
    }
    phase = (phase + 1) & 3;
  }

  void set_drive(int pwm, int dir) {
    pwm = constrain(pwm, SPEED_MIN, SPEED_MAX);
    analogWrite(RPWM, dir > 0 ? pwm : 0);
    analogWrite(LPWM, dir < 0 ? pwm : 0);
  }

  void steer_to(int deg) {
    deg = constrain(deg, SERVO_MIN, SERVO_MAX);
    servo.write(deg);
  }

  void set_steer_norm(float u) {
    u = SERVO_DIR * (u + steer_trim);
    u = constrain(u, -1.0f, 1.0f);
    float angle = (u >= 0) ? SERVO_CENTER + u * span_right : SERVO_CENTER + u * span_left;
    steer_to((int)angle);
  }

  void run_pid_turn() {
    if (state != TURNING) return;
    float err = wrap180(target_yaw - yaw_deg);
    if (fabs(err) <= TURN_TOL_DEG || (millis() - turn_start_ms) > TURN_MAX_MS) {
      set_drive(0, 0);
      set_steer_norm(0.0f);
      state = IDLE;
      pid_i = 0; prev_err = 0; d_filt = 0; last_pid_ms = 0;
      turn_pwm_override = -1; turn_dir = 1;
      buzzer_rapid = false;
#if BUZZER_ENABLED
      noTone(BUZZER_PIN);
#endif
      Serial.println("TURN_DONE");
      beep(60);
      return;
    }

    unsigned long now = millis();
    float dt = (last_pid_ms == 0) ? 0.02f : (now - last_pid_ms) / 1000.0f;
    last_pid_ms = now;
    pid_i += err * dt;
    float d = (err - prev_err) / (dt > 1e-3f ? dt : 1e-3f);
    d_filt = 0.35f * d + 0.65f * d_filt;
    prev_err = err;
    float u = 3.0f * err + 0.0f * pid_i + 0.26f * d_filt;
    float u_norm = constrain(u / 45.0f, -1.2f, 1.2f);
    set_steer_norm(turn_dir >= 0 ? u_norm : -u_norm);
    set_drive(turn_pwm_override >= 0 ? turn_pwm_override : turn_pwm, turn_dir);
    buzzer_rapid = true;
  }

  void handle_obstacle() {
    if (state != CENTERING) return;
    if (dist[0] < OBS_FRONT_CM) {
      set_drive(0, 0);
      state = TURNING;
      target_yaw = wrap180(yaw_deg + (dist[1] > dist[2] ? 90 : -90));
      turn_start_ms = millis();
      pid_i = 0; prev_err = 0; d_filt = 0; last_pid_ms = 0;
      turn_pwm_override = turn_pwm;
      turn_dir = 1;
      buzzer_rapid = true;
      Serial.println("OBS_AVOID_TURN");
    } else if (direction < 0 && dist[3] < OBS_SIDE_CM) {
      set_drive(0, 0);
      state = IDLE;
      Serial.println("OBS_AVOID_STOP");
    }
  }

  void set_armed(bool on) {
    if (on && !armed) {
      armed = true;
      beep(100);
      Serial.println("ARMED");
    } else if (!on && armed) {
      armed = false;
      state = IDLE;
      set_drive(0, 0);
      set_steer_norm(0.0f);
    }
  }

  void handle_serial(const String& cmd) {
    if (cmd.equalsIgnoreCase("PING")) {
      Serial.println("PONG");
      return;
    }
    if (cmd.equalsIgnoreCase("START") || cmd.equalsIgnoreCase("S")) {
      set_armed(true);
      return;
    }
    if (!armed && !cmd.startsWith("SET_")) return;

    if (cmd.startsWith("CENTER,")) {
      int c1 = cmd.indexOf(','), c2 = cmd.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        steer_norm = constrain(cmd.substring(c1 + 1, c2).toFloat(), -1.0f, 1.0f);
        speed = constrain(cmd.substring(c2 + 1).toInt(), SPEED_MIN, SPEED_MAX);
        direction = (speed > 0) ? 1 : 0;
        if (state != TURNING) state = CENTERING;
        buzzer_rapid = false;
#if BUZZER_ENABLED
        noTone(BUZZER_PIN);
#endif
      }
    } else if (cmd.startsWith("BACKC,")) {
      int c1 = cmd.indexOf(','), c2 = cmd.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        steer_norm = constrain(cmd.substring(c1 + 1, c2).toFloat(), -1.0f, 1.0f);
        speed = constrain(cmd.substring(c2 + 1).toInt(), SPEED_MIN, SPEED_MAX);
        direction = (speed > 0) ? -1 : 0;
        if (state != TURNING) state = CENTERING;
        buzzer_rapid = false;
#if BUZZER_ENABLED
        noTone(BUZZER_PIN);
#endif
      }
    } else if (cmd.startsWith("BACK,")) {
      int c1 = cmd.indexOf(',');
      if (c1 > 0) {
        speed = constrain(cmd.substring(c1 + 1).toInt(), SPEED_MIN, SPEED_MAX);
        direction = (speed > 0) ? -1 : 0;
        if (state != TURNING) state = CENTERING;
        buzzer_rapid = false;
#if BUZZER_ENABLED
        noTone(BUZZER_PIN);
#endif
      }
    } else if (cmd.startsWith("TURN_ABS,")) {
      int c1 = cmd.indexOf(',');
      if (c1 > 0) {
        int c2 = cmd.indexOf(',', c1 + 1);
        target_yaw = wrap180(cmd.substring(c1 + 1, c2 > 0 ? c2 : cmd.length()).toFloat());
        turn_pwm_override = -1;
        turn_dir = 1;
        if (c2 > 0) {
          String tail = cmd.substring(c2 + 1);
          int ct = tail.indexOf(',');
          if (ct < 0) {
            if (tail.equalsIgnoreCase("REV")) turn_dir = -1;
            else turn_pwm_override = tail.toInt();
          } else {
            turn_pwm_override = tail.substring(0, ct).toInt();
            if (tail.substring(ct + 1).equalsIgnoreCase("REV")) turn_dir = -1;
          }
        }
        state = TURNING;
        pid_i = 0; prev_err = 0; d_filt = 0; last_pid_ms = 0;
        turn_start_ms = millis();
        buzzer_rapid = true;
        beep(120);
      }
    } else if (cmd.startsWith("STEER_DEG,")) {
      int c1 = cmd.indexOf(',');
      if (c1 > 0) steer_to(cmd.substring(c1 + 1).toInt());
    } else if (cmd.startsWith("TRIM_NORM,")) {
      int c1 = cmd.indexOf(',');
      if (c1 > 0) {
        steer_trim = constrain(cmd.substring(c1 + 1).toFloat(), -0.3f, 0.3f);
        beep(40);
      }
    } else if (cmd.startsWith("SET_CENTER,")) {
      int c1 = cmd.indexOf(',');
      if (c1 > 0) {
        int center = constrain(cmd.substring(c1 + 1).toInt(), SERVO_MIN, SERVO_MAX);
        span_left = center - SERVO_MIN;
        span_right = SERVO_MAX - center;
        steer_to(center);
        beep(80);
      }
    } else if (cmd.startsWith("SET_TURN_PWM,")) {
      int c1 = cmd.indexOf(',');
      if (c1 > 0) {
        turn_pwm = constrain(cmd.substring(c1 + 1).toInt(), SPEED_MIN, SPEED_MAX);
        beep(60);
      }
    } else if (cmd.equalsIgnoreCase("CLEAR_TURN_PWM")) {
      turn_pwm_override = -1;
      beep(40);
    } else if (cmd.startsWith("SET_US_TIMEOUT,")) {
      int c1 = cmd.indexOf(',');
      if (c1 > 0) {
        US_TIMEOUT = (unsigned long)cmd.substring(c1 + 1).toInt();
        beep(40);
      }
    } else if (cmd.startsWith("SET_US_FAR_CM,")) {
      int c1 = cmd.indexOf(',');
      if (c1 > 0) {
        US_FAR_CM = cmd.substring(c1 + 1).toInt();
        beep(40);
      }
    } else if (cmd.equalsIgnoreCase("STOP")) {
      state = IDLE;
      speed = 0;
      direction = 0;
      set_drive(0, 0);
      set_steer_norm(0.0f);
      buzzer_rapid = false;
#if BUZZER_ENABLED
      noTone(BUZZER_PIN);
#endif
    }
  }

  void send_telemetry() {
    static unsigned long last = 0;
    unsigned long now = millis();
    if (now - last >= 16) {
      last = now;
      Serial.print("TLM,yaw="); Serial.print(yaw_deg, 1);
      Serial.print(",state="); Serial.print((int)state);
      Serial.print(",steer="); Serial.print(steer_norm, 2);
      Serial.print(",speed="); Serial.print(speed);
      Serial.print(",dF="); Serial.print(dist[0]);
      Serial.print(",dL="); Serial.print(dist[1]);
      Serial.print(",dR="); Serial.print(dist[2]);
      Serial.print(",dB="); Serial.println(dist[3]);
    }
  }

  void update() {
    chirp_rapid();
    read_sensors();
    read_imu();
    if (state == TURNING && (millis() - last_imu_ok_ms) > IMU_STALE_MS) {
      set_drive(0, 0);
      set_steer_norm(0.0f);
      state = IDLE;
      buzzer_rapid = false;
#if BUZZER_ENABLED
      noTone(BUZZER_PIN);
#endif
      Serial.println("WARN,IMU_STALE_STOP");
    }
    if (!armed && digitalRead(START_BTN) == LOW) {
      delay(20);
      if (digitalRead(START_BTN) == LOW) {
        set_armed(true);
        while (digitalRead(START_BTN) == LOW) delay(5);
      }
    }
    handle_obstacle();
    switch (state) {
      case IDLE:
        set_drive(0, 0);
        set_steer_norm(0.0f);
        break;
      case CENTERING:
        steer_filt += 0.18f * (steer_norm - steer_filt);
        speed_filt += 0.35f * ((float)speed - speed_filt);
        set_steer_norm(direction < 0 ? -steer_filt : steer_filt);
        set_drive((int)speed_filt, direction);
        break;
      case TURNING:
        run_pid_turn();
        break;
    }
    send_telemetry();
  }

private:
  float wrap180(float a) {
    while (a > 180) a -= 360;
    while (a < -180) a += 360;
    return a;
  }
};

RobotControl robot;

void setup() {
  robot.begin();
}

void loop() {
  static String rx;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (rx.length() > 0) {
        robot.handle_serial(rx);
        rx = "";
      }
    } else {
      rx += ch;
      if (rx.length() > 128) rx = "";
    }
  }
  robot.update();
}