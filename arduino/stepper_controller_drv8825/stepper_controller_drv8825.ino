#define STEP_PIN 6      //Motor X step pin
#define DIR_PIN 7       //Motor X direction pin
#define ENABLE_PIN 8    //Motor X enable pin

#define STEP_PIN2 5     //Motor Y step pin
#define DIR_PIN2 4      //Motor Y direction pin
#define ENABLE_PIN2 12  //Motor Y enable pin

long position_steps = 0;
long position_steps2 = 0;

// Motor 2 simple step-based control
bool motor2_running = false;
unsigned long motor2_step_interval_us = 3500;
unsigned long motor2_last_step_us = 0;

float accel = 2000.0;   // steps/s²
int max_speed = 2000;   // steps/s

void stepPulse(unsigned int delay_us)
{
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delay_us);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delay_us);
}

void stepPulse2()
{
  digitalWrite(STEP_PIN2, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN2, LOW);
}

void serviceMotor2()
{
  if(!motor2_running || motor2_step_interval_us == 0)
    return;

  unsigned long now = micros();

  if(now - motor2_last_step_us >= motor2_step_interval_us)
  {
    stepPulse2();
    motor2_last_step_us = now;

    if(digitalRead(DIR_PIN2) == HIGH) position_steps2++;
    else position_steps2--;
  }
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

    serviceMotor2();
    stepPulse(delay_us);

    if(dir) position_steps++;
    else position_steps--;
  }

  Serial.print("POS ");
  Serial.println(position_steps);
}

void moveSteps2(long steps, int speed)
{
  bool dir = steps >= 0;
  digitalWrite(DIR_PIN2, dir);

  long count = abs(steps);

  if(speed <= 0)
    speed = 1;

  unsigned long step_interval_us = 1000000UL / (unsigned long)speed;

  for(long i = 0; i < count; i++)
  {
    serviceMotor2();
    stepPulse2();
    delayMicroseconds(step_interval_us);

    if(dir) position_steps2++;
    else position_steps2--;
  }

  Serial.print("POS2 ");
  Serial.println(position_steps2);
}

void startMotor2(int speed)
{
  if(speed == 0)
  {
    stopMotor2();
    return;
  }

  bool dir = speed > 0;
  unsigned long abs_speed = abs(speed);

  if(abs_speed == 0)
    abs_speed = 1;

  digitalWrite(DIR_PIN2, dir);
  motor2_step_interval_us = 1000000UL / abs_speed;
  motor2_last_step_us = micros();
  motor2_running = true;
}

void stopMotor2()
{
  motor2_running = false;
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
}

void loop()
{
  serviceMotor2();

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

    else if(cmd.startsWith("MOVE2"))
    {
      long steps;
      int speed;

      sscanf(cmd.c_str(),"MOVE2 %ld %d",&steps,&speed);
      moveSteps2(steps, speed);
    }

    else if(cmd.startsWith("START2"))
    {
      int speed = 200;
      sscanf(cmd.c_str(),"START2 %d",&speed);
      startMotor2(speed);
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
