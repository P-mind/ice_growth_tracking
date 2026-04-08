#define STEP_PIN 6      //Motor X step pin
#define DIR_PIN 7       //Motor X direction pin
#define ENABLE_PIN 8    //Motor X enable pin

#define STEP_PIN2 5     //Motor Y step pin
#define DIR_PIN2 4      //Motor Y direction pin
#define ENABLE_PIN2 12  //Motor Y enable pin

long position_steps = 0;
long position_steps2 = 0;

// Timer2 interrupt variables for motor 2 at 60 RPM (200 steps/sec)
volatile int step_pin_state2 = LOW;

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

// Timer2 ISR - drives motor 2 at 60 RPM (200 steps/sec)
// Timer2 fires at 400 Hz, toggling step pin to create 200 complete pulses/sec
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
  
  // Configure Timer2 for 400 Hz interrupt (drives motor 2 at 60 RPM = 200 steps/sec)
  // Timer2 is 8-bit, so with prescaler 64 and compare value 62:
  // Frequency = 16,000,000 / (64 * (62 + 1)) = 16,000,000 / 4,032 ≈ 3,968 Hz
  // Adjust to approximately 400 Hz for 200 pulses/sec at double toggle
  TCCR2A = 0x02;  // CTC mode
  TCCR2B = 0x04;  // Prescaler 64
  OCR2A = 62;     // Compare value
  TIMSK2 = 0x02;  // Enable compare interrupt
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