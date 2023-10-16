
  int in1 = 4;  //right
  int in2 = 5;  //right
  int in3 = 6;
  int in4 = 7;
  int enA = 11;
  int enB = 10; 
  int ir1 = 8;
  int ir2 = 9; 
  int ir3 = 12;
  int ir4 = 13;
  int sensor1;
  int sensor2; 
  int sensor3;
  int sensor4;
void setup() {
  //l298
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT);
  pinMode (in3, OUTPUT);
  pinMode (in4, OUTPUT);
  pinMode (enA, OUTPUT);
  pinMode (enB, OUTPUT);
  //sensor
  pinMode (ir1, INPUT);
  pinMode (ir2, INPUT);
  pinMode (ir3, INPUT);
  pinMode (ir4, INPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
  readsensor();
  if ((sensor1 == 0)&&(sensor2 == 0)&&(sensor3 == 0)&&(sensor4 == 0)) {
    forward();
  };
    if ((sensor1 == 1)&&(sensor2 == 1)&&(sensor3 == 1)&&(sensor4 == 1)) {
    reverse();
  };
  
  //delay(4000);
  /*
  left();
  delay(2000);
  reverse();
  delay(2000);
  right();
  delay(2000);
  */
}

void forward() {
  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  digitalWrite (in3, HIGH);
  digitalWrite (in4, LOW);
  analogWrite (enA, 126);
  analogWrite (enB, 100);
}

void reverse() {
  digitalWrite (in1, LOW);
  digitalWrite (in2, HIGH);
  digitalWrite (in3, LOW);
  digitalWrite (in4, HIGH);
  analogWrite (enA, 130);
  analogWrite (enB, 130);
}

void left() {
  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  digitalWrite (in3, LOW);
  digitalWrite (in4, HIGH);
  analogWrite (enA, 130);
  analogWrite (enB, 130);
}

void right() {
  digitalWrite (in1, LOW);
  digitalWrite (in2, HIGH);
  digitalWrite (in3, HIGH);
  digitalWrite (in4, LOW);
  analogWrite (enA, 130);
  analogWrite (enB, 130);
}
void readsensor() {
  sensor1 = digitalRead(ir1);
  sensor2 = digitalRead(ir2);
  sensor3 = digitalRead(ir3);
  sensor4 = digitalRead(ir4);
}