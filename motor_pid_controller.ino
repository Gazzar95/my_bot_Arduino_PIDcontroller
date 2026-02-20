/*
MDDS30 (PWM+DIR) Dual Motor Velocity PI Controller (Arduino Nano)

WIRING (Arduino -> MDDS30)
Right motor (Channel 2):
  D5  -> AN2 (PWM_R)
  D6  -> IN2 (DIR_R)

Left motor (Channel 1):
  D9  -> AN1 (PWM_L)
  D10 -> IN1 (DIR_L)

ENCODERS (Motor -> Arduino)
Right encoder:
  A -> D2 (interrupt), B -> D4
Left encoder:
  A -> D3 (interrupt), B -> A5

SERIAL @ 57600
  s<N> : set BOTH targets (ticks/sec)
  r<N> : set RIGHT target
  l<N> : set LEFT target
  x    : stop + reset integrators

NOTES
- Velocity is measured in "encoder A rising edges per second" (ticks/sec).
- If a motor runs away / pins to 255, your feedback sign is wrong:
  fix by flipping ENCODER_SIGN_* (NOT just motor direction).
*/

#include <Arduino.h>

/* ----- MDDS30 pins ----- */
#define PWM_R 5  // AN2
#define DIR_R 6  // IN2
#define PWM_L 9  // AN1
#define DIR_L 10 // IN1

/* ----- Encoder pins ----- */
#define ENC_R_A 3
#define ENC_R_B A5
#define ENC_L_A 2
#define ENC_L_B 4

/* ========= HARD-CODED DIRECTION / SIGN =========
   These are the ONLY things you should flip to fix direction issues.
   Goal convention: "Forward command => robot forward => measured velocity positive"
*/
const bool MOTOR_INVERT_RIGHT = false; // flips DIR meaning for right motor
const bool MOTOR_INVERT_LEFT = true;   // flips DIR meaning for left motor
const bool ENCODER_SIGN_RIGHT = true;  // flips measured velocity sign for right
const bool ENCODER_SIGN_LEFT = true;   // flips measured velocity sign for left

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
const unsigned long DT_MS = 50; // 50 ms loop (stable on Nano)
const float DT = 0.050f;

/* ----- PI gains (start conservative) ----- */
float Kp = 0.12f;
float Ki = 0.12f;
float Kd = 0.00f; // unused unless you enable it

/* ----- Targets (ticks/sec) ----- */
float target_r = 0;
float target_l = 0;

/* ----- PI state ----- */
float integ_r = 0, integ_l = 0;
float prev_err_r = 0, prev_err_l = 0;

/* ----- Limits ----- */
const float PWM_MAX = 2000.0f;
const float INTEG_MAX = 2000.0f;

/* ----- Start assist (stiction) ----- */
const int START_PWM_R = 35;
const int START_PWM_L = 35;
const float START_VEL_THRESH = 5.0f; // ticks/sec considered "not moving"

/* ----- Filtering ----- */
static float vel_r_f = 0.0f, vel_l_f = 0.0f;
const float alpha = 0.35f; // 0..1 higher = less smoothing

unsigned long last_print = 0;

/* Safely snapshot volatile encoder counts */
static inline void readEncoders(long &r, long &l)
{
  noInterrupts();
  r = enc_r;
  l = enc_l;
  interrupts();
}

/* Drive motor with signed PWM.
   pwmSigned > 0 => forward, < 0 => reverse
*/
static inline void driveMotor(int pwmPin, int dirPin, float pwmSigned, bool invertMotor)
{
  pwmSigned = constrain(pwmSigned, -PWM_MAX, PWM_MAX);

  bool forward = (pwmSigned >= 0);
  int mag = (int)fabs(pwmSigned);

  if (invertMotor)
    forward = !forward;

  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, mag);
}

/* Serial commands: s200, r150, l-120, x */
void handleSerial()
{
  if (!Serial.available())
    return;

  char c = Serial.read();

  if (c == 'x')
  {
    target_r = target_l = 0;
    integ_r = integ_l = 0;
    prev_err_r = prev_err_l = 0;
    driveMotor(PWM_R, DIR_R, 0, MOTOR_INVERT_RIGHT);
    driveMotor(PWM_L, DIR_L, 0, MOTOR_INVERT_LEFT);
    while (Serial.available())
      Serial.read();
    Serial.println("STOP");
    return;
  }

  long v = Serial.parseInt();

  if (c == 's')
  {
    target_r = (float)v;
    target_l = (float)v;
  }
  else if (c == 'r')
  {
    target_r = (float)v;
  }
  else if (c == 'l')
  {
    target_l = (float)v;
  }

  while (Serial.available())
    Serial.read();
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

  Serial.begin(57600);
  delay(300);

  driveMotor(PWM_R, DIR_R, 0, MOTOR_INVERT_RIGHT);
  driveMotor(PWM_L, DIR_L, 0, MOTOR_INVERT_LEFT);

  Serial.println("MDDS30 PI velocity controller ready.");
  Serial.println("Commands: sN, rN, lN, x");
}

void loop()
{
  static unsigned long last_tick = 0;
  handleSerial();

  unsigned long now = millis();
  if (now - last_tick < DT_MS)
    return;
  last_tick += DT_MS;

  /* Encoder deltas -> velocity */
  long r_now, l_now;
  readEncoders(r_now, l_now);

  static long r_prev = 0, l_prev = 0;
  long dr = r_now - r_prev;
  long dl = l_now - l_prev;
  r_prev = r_now;
  l_prev = l_now;

  float vel_r = dr / DT;
  float vel_l = dl / DT;

  if (ENCODER_SIGN_RIGHT)
    vel_r = -vel_r;
  if (ENCODER_SIGN_LEFT)
    vel_l = -vel_l;

  /* Filter */
  vel_r_f = alpha * vel_r + (1.0f - alpha) * vel_r_f;
  vel_l_f = alpha * vel_l + (1.0f - alpha) * vel_l_f;

  /* Errors */
  float err_r = (target_r - vel_r_f);
  float err_l = target_l - vel_l_f;

  /* Derivative (optional) */
  float derr_r = (err_r - prev_err_r) / DT;
  float derr_l = (err_l - prev_err_l) / DT;
  prev_err_r = err_r;
  prev_err_l = err_l;

  // disable integral when far from target
  float Ki_eff = Ki;
  //  if (fabs(err_r) > 100.0f)
  //  { // tune 150 up/down
  //    Ki_eff = 0.10f;
  //  }

  // PID control
  float pwm_r_raw = Kp * err_r + Ki_eff * integ_r + Kd * derr_r;
  float pwm_l_raw = Kp * err_l + Ki_eff * integ_l + Kd * derr_l;
  /* Saturate */
  float pwm_r = constrain(pwm_r_raw, -PWM_MAX, PWM_MAX);
  float pwm_l = constrain(pwm_l_raw, -PWM_MAX, PWM_MAX);

  /* Anti-windup: only integrate if not pushing further into saturation */
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

  /* Recompute after updated integrator */
  pwm_r_raw = Kp * err_r + Ki * integ_r + Kd * derr_r;
  pwm_l_raw = Kp * err_l + Ki * integ_l + Kd * derr_l;
  pwm_r = constrain(pwm_r_raw, -PWM_MAX, PWM_MAX);
  pwm_l = constrain(pwm_l_raw, -PWM_MAX, PWM_MAX);

  /* Start assist (helps both start together) */
  if (fabs(target_r) > 1.0f && fabs(vel_r_f) < START_VEL_THRESH)
  {
    if (pwm_r > 0)
      pwm_r = max(pwm_r, (float)START_PWM_R);
    if (pwm_r < 0)
      pwm_r = min(pwm_r, -(float)START_PWM_R);
  }
  if (fabs(target_l) > 1.0f && fabs(vel_l_f) < START_VEL_THRESH)
  {
    if (pwm_l > 0)
      pwm_l = max(pwm_l, (float)START_PWM_L);
    if (pwm_l < 0)
      pwm_l = min(pwm_l, -(float)START_PWM_L);
  }

  /* Drive motors */
  driveMotor(PWM_R, DIR_R, pwm_r, MOTOR_INVERT_RIGHT);
  driveMotor(PWM_L, DIR_L, pwm_l, MOTOR_INVERT_LEFT);

  /* Debug @ 10 Hz */
  if (now - last_print >= 100)
  {
    last_print = now;

    Serial.print("tr ");
    Serial.print(target_r);

    Serial.print(" vr ");
    Serial.print(vel_r_f);

    //    Serial.print(" er ");
    //    Serial.println(err_r);

    // Serial.print(" ir ");
    // Serial.print(integ_r);

    Serial.print(" tl ");
    Serial.print(target_l);

    Serial.print(" vl ");
    Serial.println(vel_l_f);

    // Serial.print(" el ");
    // Serial.print(err_l);

    // Serial.print(" il ");
    // Serial.println(integ_l);
  }
}
