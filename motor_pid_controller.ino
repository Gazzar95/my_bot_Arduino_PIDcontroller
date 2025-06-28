// ===== Arduino Motor Controller with PID =====
// This code controls 2 motors using BTS7960 drivers and reads encoders.
// It also uses a simple PID controller to try to keep the motor at a target speed.

// ----- Pin Definitions -----
// Right motor driver pins
#define RPWM_R 5
#define LPWM_R 6
#define R_EN   7

// Left motor driver pins
#define RPWM_L 9
#define LPWM_L 10
#define L_EN   11

// Encoder pins (RIGHT motor)
#define ENC_R_A 2   // Must be interrupt pin
#define ENC_R_B 4

// Encoder pins (LEFT motor)
#define ENC_L_A 3   // Must be interrupt pin
#define ENC_L_B A5

// ----- Variables -----
volatile long encoderRight = 0;  // Counts from right encoder
volatile long encoderLeft = 0;   // Counts from left encoder

// ----- PID Control Variables -----
float targetSpeedRight = 100;  // Target speed in ticks per second (set by user)
float currentSpeedRight = 0;
float pwmRight = 0;            // PWM value sent to motor

float kp = 1.0;  // PID tuning constants
float ki = 0.5;
float kd = 0.0;
float integral = 0;
float lastError = 0;

unsigned long lastPIDTime = 0;
long lastEncoderRight = 0;

void setup() {
  Serial.begin(57600);

  // Setup motor driver pins
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);  // Enable driver

  // Setup encoder pins
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  // Attach interrupt to RIGHT encoder A channel
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), updateEncoderRight, CHANGE);

  // Initialize PID timer
  lastPIDTime = millis();
}

void loop() {
  // ----- Serial Commands (Optional) -----
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's') {
      targetSpeedRight = Serial.parseInt();
      Serial.print("Target speed set to: ");
      Serial.println(targetSpeedRight);
    }
  }

  // ----- PID Control Loop -----
  unsigned long now = millis();
  if (now - lastPIDTime >= 100) {  // Run PID every 100ms
    long encoderCounts = encoderRight - lastEncoderRight;
    currentSpeedRight = encoderCounts / 0.1;  // Ticks per second

    float error = targetSpeedRight - currentSpeedRight;
    integral += error * 0.1;  // 0.1 sec interval
    float derivative = (error - lastError) / 0.1;

    // Simple PID formula
    float output = kp * error + ki * integral + kd * derivative;
    pwmRight += output;

    // Constrain PWM value
    pwmRight = constrain(pwmRight, -255, 255);

    // Send to motor driver
    driveMotor(RPWM_R, LPWM_R, pwmRight);

    // Save values for next loop
    lastEncoderRight = encoderRight;
    lastError = error;
    lastPIDTime = now;

    // Debug print
    Serial.print("Speed: "); Serial.print(currentSpeedRight);
    Serial.print(" PWM: "); Serial.println(pwmRight);
  }
}

// ----- Functions -----

// Drive motor given PWM value (positive = forward, negative = backward)
void driveMotor(int rpwm, int lpwm, float pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(rpwm, pwm);
    analogWrite(lpwm, 0);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -pwm);
  }
}

// Interrupt routine to update RIGHT encoder count
void updateEncoderRight() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B))
    encoderRight++;
  else
    encoderRight--;
}
