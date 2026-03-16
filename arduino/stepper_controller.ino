#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4

long position_steps = 0;

void setup() {

  pinMode(STEP_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(ENABLE_PIN,OUTPUT);

  digitalWrite(ENABLE_PIN,LOW);

  Serial.begin(115200);
}

void moveSteps(long steps,int speed){

  bool dir = steps>=0;
  digitalWrite(DIR_PIN,dir);

  long count = abs(steps);
  int delay_us = 1000000/(speed*2);

  for(long i=0;i<count;i++){

    digitalWrite(STEP_PIN,HIGH);
    delayMicroseconds(delay_us);

    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(delay_us);

    if(dir) position_steps++;
    else position_steps--;
  }

  Serial.print("POS ");
  Serial.println(position_steps);
}

void loop(){

  if(Serial.available()){

    String cmd = Serial.readStringUntil('\n');

    if(cmd.startsWith("MOVE")){

      long steps;
      int speed;

      sscanf(cmd.c_str(),"MOVE %ld %d",&steps,&speed);

      moveSteps(steps,speed);
    }

    if(cmd.startsWith("POS?")){

      Serial.print("POS ");
      Serial.println(position_steps);
    }
  }
}