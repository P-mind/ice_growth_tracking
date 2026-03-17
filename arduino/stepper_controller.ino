#include <TMCStepper.h>

// ---------------- PIN CONFIG ----------------
#define STEP_PIN 6
#define DIR_PIN 7
#define EN_PIN 8

#define SERIAL_PORT Serial
#define DRIVER_ADDRESS 0b00

#define R_SENSE 0.11f

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

// ---------------- SYSTEM PARAMETERS ----------------
long position_steps = 0;

// Motion tuning (optimized for camera stability)
float accel = 1500.0;     // lower acceleration = less vibration
int max_speed = 1500;     // steps/sec (safe for precision)

// ---------------- STEP FUNCTION ----------------
void stepPulse(unsigned int delay_us)
{
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delay_us);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delay_us);
}

// ---------------- MOTION CONTROL ----------------
void moveSteps(long steps, int target_speed)
{
  bool dir = steps >= 0;
  digitalWrite(DIR_PIN, dir);

  long count = abs(steps);

  float v = 200.0;  // start speed (soft start)
  float delay_us;

  for(long i = 0; i < count; i++)
  {
    // Acceleration ramp
    if(v < target_speed)
      v += accel * 0.0005;

    if(v > max_speed)
      v = max_speed;

    delay_us = 1000000.0 / v / 2.0;

    stepPulse(delay_us);

    if(dir) position_steps++;
    else position_steps--;
  }

  Serial.print("POS ");
  Serial.println(position_steps);
}

// ---------------- SETUP ----------------
void setup()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW); // enable driver

  SERIAL_PORT.begin(115200);

  // -------- TMC2209 INIT --------
  driver.begin();

  driver.toff(5);

  // CRITICAL: tuned for Adafruit NEMA17
  driver.rms_current(300);  // mA (safe default)

  driver.microsteps(32);     // high precision

  driver.en_spreadCycle(false); // stealthChop (quiet)
  driver.pwm_autoscale(true);

  // Optional: reduce idle current (less heating)
  driver.idle_current_percent(50);

  Serial.println("TMC2209 + Adafruit NEMA17 ready");
}

// ---------------- LOOP ----------------
void loop()
{
  if(Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');

    if(cmd.startsWith("MOVE"))
    {
      long steps;
      int speed;

      sscanf(cmd.c_str(), "MOVE %ld %d", &steps, &speed);

      moveSteps(steps, speed);
    }

    else if(cmd.startsWith("POS?"))
    {
      Serial.print("POS ");
      Serial.println(position_steps);
    }

    else if(cmd.startsWith("ZERO"))
    {
      position_steps = 0;
      Serial.println("ZEROED");
    }
  }
}