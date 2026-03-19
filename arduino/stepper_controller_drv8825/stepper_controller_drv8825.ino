#define STEP_PIN 6
#define DIR_PIN 7
#define ENABLE_PIN 8

long position_steps = 0;

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

void setup()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);

  Serial.begin(115200);
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

    else if(cmd.startsWith("ZERO"))
    {
      position_steps = 0;
    }
  }
}