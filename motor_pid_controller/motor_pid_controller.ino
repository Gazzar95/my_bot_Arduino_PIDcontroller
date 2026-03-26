/*
MDDS30 (PWM+DIR) Dual Motor Velocity PID Controller (Arduino Nano)

ROS 2 SERIAL PROTOCOL (matches mobile_robot_hardware plugin)
- RX command: CMD <left_vel_rad_s> <right_vel_rad_s>
- TX feedback: FB <left_vel_rad_s> <right_vel_rad_s> <left_pos_rad> <right_pos_rad>

LEGACY MANUAL COMMANDS (for Serial Monitor/Plotter troubleshooting)
- s<N> : set BOTH targets (N in ticks/sec)
- l<N> : set LEFT target (N in ticks/sec)
- r<N> : set RIGHT target (N in ticks/sec)
- x    : stop + reset integrators

WIRING (Arduino -> MDDS30)
Right motor (Channel 2):
  D5  -> AN2 (PWM_R)
  D6  -> IN2 (DIR_R)
Left motor (Channel 1):
  D9  -> AN1 (PWM_L)
  D10 -> IN1 (DIR_L)

ENCODERS (Motor -> Arduino)
Right encoder:
  A -> D3 (interrupt), B -> A5
Left encoder:
  A -> D2 (interrupt), B -> D4
*/

#include <Arduino.h>

/* ----- MDDS30 pins ----- */
#define PWM_R 5
#define DIR_R 6
#define PWM_L 9
#define DIR_L 10

/* ----- Encoder pins ----- */
#define ENC_R_A 3
#define ENC_R_B A5
#define ENC_L_A 2
#define ENC_L_B 4

/* ========= HARD-CODED DIRECTION / SIGN ========= */
const bool MOTOR_INVERT_RIGHT = false;
const bool MOTOR_INVERT_LEFT = false;
const bool ENCODER_SIGN_RIGHT = true;
const bool ENCODER_SIGN_LEFT = false;

/* ----- Encoder conversion -----
Set this to encoder counts-per-wheel-revolution (after gearbox).
*/
const float TICKS_PER_REV_RIGHT = 600.0f;
const float TICKS_PER_REV_LEFT = 600.0f;
const float TWO_PI_F = 6.28318530718f;

/* ----- Encoder counts ----- */
volatile long enc_r = 0;
volatile long enc_l = 0;

void ISR_enc_r()
{
  if (digitalRead(ENC_R_B))
    enc_r++;
  else
    enc_r--;
}

void ISR_enc_l()
{
  if (digitalRead(ENC_L_B))
    enc_l++;
  else
    enc_l--;
}

/* ----- Control timing ----- */
const unsigned long DT_MS = 50; // 20 Hz loop
const float DT = 0.050f;

/* ----- PID gains ----- */
// These gains are scaled for rad/s control (roughly old_ticks gains * TICKS_PER_REV / 2pi).
float Kp = 11.5f;
float Ki = 11.5f;
float Kd = 0.00f;

/* ----- Targets (rad/s) ----- */
float target_r = 0.0f;
float target_l = 0.0f;

/* ----- PID state ----- */
float integ_r = 0.0f, integ_l = 0.0f;
float prev_err_r = 0.0f, prev_err_l = 0.0f;

/* ----- Limits ----- */
const float PWM_MAX = 2000.0f;
const float INTEG_MAX = 2000.0f;

/* ----- Start assist ----- */
const int START_PWM_R = 35;
const int START_PWM_L = 35;
const float START_VEL_THRESH_RAD_S = 0.052f;

/* ----- Filtering ----- */
static float vel_r_f = 0.0f, vel_l_f = 0.0f;
const float alpha = 0.35f;

/* ----- Serial input buffer ----- */
String rx_line;

/* ----- Position integration from encoder counts ----- */
float pos_r_rad = 0.0f;
float pos_l_rad = 0.0f;

static inline void readEncoders(long &r, long &l)
{
  noInterrupts();
  r = enc_r;
  l = enc_l;
  interrupts();
}

static inline void driveMotor(int pwmPin, int dirPin, float pwmSigned, bool invertMotor)
{
  pwmSigned = constrain(pwmSigned, -PWM_MAX, PWM_MAX);
  bool forward = (pwmSigned >= 0.0f);
  int mag = (int)fabs(pwmSigned);

  if (invertMotor)
    forward = !forward;

  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, mag);
}

float ticksToRad(long ticks, float ticks_per_rev)
{
  return ((float)ticks) * TWO_PI_F / ticks_per_rev;
}

void stopMotors()
{
  target_r = 0.0f;
  target_l = 0.0f;
  integ_r = 0.0f;
  integ_l = 0.0f;
  prev_err_r = 0.0f;
  prev_err_l = 0.0f;
  driveMotor(PWM_R, DIR_R, 0.0f, MOTOR_INVERT_RIGHT);
  driveMotor(PWM_L, DIR_L, 0.0f, MOTOR_INVERT_LEFT);
}

void handleProtocolLine(const String &line)
{
  if (line.length() == 0)
    return;

  if (line == "STOP" || line == "x")
  {
    stopMotors();
    return;
  }

  if (!line.startsWith("CMD "))
  {
    return;
  }

  // Parse with String methods to avoid AVR sscanf(%f) limitations.
  String payload = line.substring(4); // after "CMD "
  payload.trim();

  int sp = payload.indexOf(' ');
  if (sp < 0)
  {
    return;
  }

  String s_left = payload.substring(0, sp);
  String s_right = payload.substring(sp + 1);
  s_left.trim();
  s_right.trim();
  if (s_left.length() == 0 || s_right.length() == 0)
  {
    return;
  }

  float left = s_left.toFloat();
  float right = s_right.toFloat();

  // Keep ordering aligned with ros2_control:
  // CMD <left> <right>
  target_l = left;
  target_r = right;
}

void handleSerial()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      handleProtocolLine(rx_line);
      rx_line = "";
    }
    else
    {
      rx_line += c;
      if (rx_line.length() > 96)
      {
        rx_line = "";
      }
    }
  }
}

void setup()
{
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_enc_r, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), ISR_enc_l, RISING);

  Serial.begin(115200);
  delay(200);

  stopMotors();
}

void loop()
{
  static unsigned long last_tick = 0;
  static long r_prev = 0, l_prev = 0;

  handleSerial();

  unsigned long now = millis();
  if (now - last_tick < DT_MS)
    return;
  last_tick += DT_MS;

  long r_now, l_now;
  readEncoders(r_now, l_now);

  long dr = r_now - r_prev;
  long dl = l_now - l_prev;
  r_prev = r_now;
  l_prev = l_now;

  // Convert encoder count delta to wheel angular velocity (rad/s).
  float vel_r = ticksToRad(dr, TICKS_PER_REV_RIGHT) / DT;
  float vel_l = ticksToRad(dl, TICKS_PER_REV_LEFT) / DT;

  if (ENCODER_SIGN_RIGHT)
    vel_r = -vel_r;
  if (ENCODER_SIGN_LEFT)
    vel_l = -vel_l;

  vel_r_f = alpha * vel_r + (1.0f - alpha) * vel_r_f;
  vel_l_f = alpha * vel_l + (1.0f - alpha) * vel_l_f;

  pos_r_rad = ticksToRad(r_now, TICKS_PER_REV_RIGHT);
  pos_l_rad = ticksToRad(l_now, TICKS_PER_REV_LEFT);
  if (ENCODER_SIGN_RIGHT)
    pos_r_rad = -pos_r_rad;
  if (ENCODER_SIGN_LEFT)
    pos_l_rad = -pos_l_rad;

  float err_r = target_r - vel_r_f;
  float err_l = target_l - vel_l_f;

  float derr_r = (err_r - prev_err_r) / DT;
  float derr_l = (err_l - prev_err_l) / DT;
  prev_err_r = err_r;
  prev_err_l = err_l;

  float pwm_r_raw = Kp * err_r + Ki * integ_r + Kd * derr_r;
  float pwm_l_raw = Kp * err_l + Ki * integ_l + Kd * derr_l;

  float pwm_r = constrain(pwm_r_raw, -PWM_MAX, PWM_MAX);
  float pwm_l = constrain(pwm_l_raw, -PWM_MAX, PWM_MAX);

  bool sat_r = (fabs(pwm_r_raw) > PWM_MAX);
  bool sat_l = (fabs(pwm_l_raw) > PWM_MAX);

  if (!(sat_r && ((pwm_r > 0 && err_r > 0) || (pwm_r < 0 && err_r < 0))))
  {
    integ_r = constrain(integ_r + err_r * DT, -INTEG_MAX, INTEG_MAX);
  }
  if (!(sat_l && ((pwm_l > 0 && err_l > 0) || (pwm_l < 0 && err_l < 0))))
  {
    integ_l = constrain(integ_l + err_l * DT, -INTEG_MAX, INTEG_MAX);
  }

  pwm_r_raw = Kp * err_r + Ki * integ_r + Kd * derr_r;
  pwm_l_raw = Kp * err_l + Ki * integ_l + Kd * derr_l;
  pwm_r = constrain(pwm_r_raw, -PWM_MAX, PWM_MAX);
  pwm_l = constrain(pwm_l_raw, -PWM_MAX, PWM_MAX);

  if (fabs(target_r) > 0.1f && fabs(vel_r_f) < START_VEL_THRESH_RAD_S)
  {
    if (pwm_r > 0)
      pwm_r = max(pwm_r, (float)START_PWM_R);
    if (pwm_r < 0)
      pwm_r = min(pwm_r, -(float)START_PWM_R);
  }
  if (fabs(target_l) > 0.1f && fabs(vel_l_f) < START_VEL_THRESH_RAD_S)
  {
    if (pwm_l > 0)
      pwm_l = max(pwm_l, (float)START_PWM_L);
    if (pwm_l < 0)
      pwm_l = min(pwm_l, -(float)START_PWM_L);
  }

  driveMotor(PWM_R, DIR_R, pwm_r, MOTOR_INVERT_RIGHT);
  driveMotor(PWM_L, DIR_L, pwm_l, MOTOR_INVERT_LEFT);

  // Feedback line consumed by MobileRobotHardware::parse_feedback():
  // FB <left_vel_rad_s> <right_vel_rad_s> <left_pos_rad> <right_pos_rad>
  Serial.print("FB ");
  Serial.print(vel_l_f, 6);
  Serial.print(' ');
  Serial.println(vel_r_f, 6);
//  Serial.println(' ');
//  Serial.print(pos_l_rad, 6);
//  Serial.print(' ');
//  Serial.println(pos_r_rad, 6);
}
