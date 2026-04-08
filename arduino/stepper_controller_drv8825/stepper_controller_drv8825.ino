#define STEP_PIN 6      //Motor X step pin
#define DIR_PIN 7       //Motor X direction pin
#define ENABLE_PIN 8    //Motor X enable pin

#define STEP_PIN2 5     //Motor Y step pin
#define DIR_PIN2 4      //Motor Y direction pin
#define ENABLE_PIN2 12  //Motor Y enable pin

long position_steps = 0;
long position_steps2 = 0;

// Motor 2 timer state and RPM control
volatile int step_pin_state2 = LOW;
float motor2_rpm = 0.0;
bool motor2_enabled = false;

float accel = 2000.0;   // steps/s²
int max_speed = 2000;   // steps/s

void stepPulse(unsigned int delay_us)
{
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delay_us);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delay_us);
}

void moveSteps(long steps, int speed)
{
  bool dir = steps >= 0;
  digitalWrite(DIR_PIN, dir);

  long count = abs(steps);

  float v = 200.0;
  float delay_us;

  for(long i=0;i<count;i++)
  {
    if(v < speed)
      v += accel * 0.0005;

    delay_us = 1000000.0 / v / 2.0;

    stepPulse(delay_us);

    if(dir) position_steps++;
    else position_steps--;
  }

  Serial.print("POS ");
  Serial.println(position_steps);
}

void setMotor2Rpm(int rpm)
{
  if(rpm < 0) rpm = 0;

  motor2_rpm = rpm;

  if(rpm == 0)
  {
    TIMSK2 &= ~_BV(OCIE2A);
    return;
  }

  float pulses_per_sec = rpm * 200.0 / 60.0 * 2.0;
  float ocr = 16000000.0 / (8.0 * pulses_per_sec) - 1.0;

  if(ocr < 0.0)
    ocr = 0.0;
  if(ocr > 255.0)
    ocr = 255.0;

  OCR2A = (uint8_t)(ocr + 0.5);

  if(motor2_enabled)
  {
    TIMSK2 |= _BV(OCIE2A);
  }
}

void startMotor2()
{
  if(motor2_rpm <= 0.0)
    return;

  motor2_enabled = true;
  TCCR2B = _BV(CS21);  // Timer2 prescaler 8
  TIMSK2 |= _BV(OCIE2A);
}

void stopMotor2()
{
  motor2_enabled = false;
  TIMSK2 &= ~_BV(OCIE2A);
  TCCR2B = 0;
}

// Timer2 ISR - drives motor 2 via set RPM
ISR(TIMER2_COMPA_vect)
{
  step_pin_state2 = !step_pin_state2;
  digitalWrite(STEP_PIN2, step_pin_state2);
  
  if(step_pin_state2 == LOW)
  {
    position_steps2++;
  }
}

void setup()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(ENABLE_PIN2, LOW);
  
  // Set direction for motor 2 (positive direction)
  digitalWrite(DIR_PIN2, HIGH);

  Serial.begin(115200);
  
  // Configure Timer2 in CTC mode, but do not enable it until the Python script starts motor 2.
  TCCR2A = _BV(WGM21);  // CTC mode
  TCCR2B = 0;           // Timer stopped until START2 command
  TIMSK2 = 0;           // No interrupt yet
}

void loop()
{
  if(Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');

    if(cmd.startsWith("MOVE"))
    {
      long steps;
      int speed;

      sscanf(cmd.c_str(),"MOVE %ld %d",&steps,&speed);

      moveSteps(steps,speed);
    }

    else if(cmd.startsWith("POS?"))
    {
      Serial.print("POS ");
      Serial.println(position_steps);
    }
    
    else if(cmd.startsWith("POS2?"))
    {
      Serial.print("POS2 ");
      Serial.println(position_steps2);
    }

    else if(cmd.startsWith("SETRPM"))
    {
      int rpm = 0;
      sscanf(cmd.c_str(),"SETRPM %d",&rpm);
      setMotor2Rpm(rpm);
      Serial.println("RPM OK");
    }

    else if(cmd.startsWith("START2"))
    {
      startMotor2();
      Serial.println("START2 OK");
    }

    else if(cmd.startsWith("STOP2"))
    {
      stopMotor2();
      Serial.println("STOP2 OK");
    }

    else if(cmd.startsWith("RESET"))
    {
      Serial.println("RESET OK");
    }

    else if(cmd.startsWith("ZERO"))
    {
      position_steps = 0;
    }
    
    else if(cmd.startsWith("ZERO2"))
    {
      position_steps2 = 0;
    }
  }
}