/*
============================================================
Encoder Bring-Up Test Firmware
============================================================

HARDWARE CONFIGURATION
---------------------
MCU:
- Arduino Nano (ATmega328P, 5V logic)

Motor Driver:
- Cytron MDDS30 SmartDriveDuo-30
  (NOT powered during this test — logic only)

Motors:
- 12V DC gear motors with quadrature encoders
  - Motor power: red/white (unused in this test)
  - Encoder A: yellow
  - Encoder B: green
  - Encoder VCC: blue (5V from Arduino)
  - Encoder GND: black

ENCODER PIN ASSIGNMENT
---------------------
Right Motor Encoder:
- Encoder A → D2 (interrupt)
- Encoder B → D4

Left Motor Encoder:
- (not used in this test)

POWERING
--------
- Arduino powered via USB
- No battery connected
- MDDS30 motor power disconnected

PURPOSE
-------
- Validate encoder wiring
- Verify direction (A/B phase)
- Confirm clean counts without motor noise

NOTES
-----
- Wheel should be rotated by hand
- Counts should increase in one direction and decrease in the other
- No motor movement should occur

============================================================
*/

#define PWM_R 5
#define DIR_R 6
#define ENC_R_A 2 // interrupt pin
#define ENC_R_B 4

volatile long enc = 0;

void ISR_enc()
{
    // Basic quadrature decode using B state at A rising edge
    if (digitalRead(ENC_R_B))
        enc++;
    else
        enc--;
}

void setup()
{
    pinMode(PWM_R, OUTPUT);
    pinMode(DIR_R, OUTPUT);

    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_enc, RISING);

    Serial.begin(57600);

    // Choose a direction for the sweep
    digitalWrite(DIR_R, HIGH);
    analogWrite(PWM_R, 0);
}

void loop()
{
    for (int pwm = 40; pwm <= 150; pwm += 10)
    {
        enc = 0;
        analogWrite(PWM_R, pwm);
        delay(1000);
        analogWrite(PWM_R, 0);

        Serial.print("PWM ");
        Serial.print(pwm);
        Serial.print(" -> ticks/sec ");
        Serial.println(enc);

        delay(2000);
    }

    // Stop after one sweep; comment this out if you want it to repeat forever
    while (true)
    {
        analogWrite(PWM_R, 0);
        delay(1000);
    }
}
