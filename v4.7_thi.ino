 #include <PID_v1.h>

// IR Sensors
int sensor1 = 7;      // Left most sensor
int sensor2 = 8;
int sensor3 = 9;
int sensor4 = 10;
int sensor5 = 11;   // Right most sensor
     
// Initial Values of Sensors
int sensor[5] = {0, 0, 0, 0, 0};

// Motor Variables              
int motorInput1 = 6;      //Motor Phải
int motorInput2 = 4;
int motorInput3 = 5;      //Motor Trái
int motorInput4 = 3;                 

//Initial Speed of Motor
double initial_motor_speed = 180; // 160-170
double motor_speed = initial_motor_speed ;

// Tốc độ rẽ
int banh_chinh = 75;//95-70
int banh_phu = 255-160; //Đảo ngược-110-130
//vọt lố - tăng phụ, giảm chính
//chưa tới - tăng chính giảm phụ

// PID Constants
double Kp = 18;//-17.5-18.0000-15-16
double Ki = 0.0;//-0.02-0.04-00
double Kd = 10.5;//13-50-75-70-90-100-205-208-222
//con rắn-tăng kd
double T = 12  ; // chuẩn
double T1 = 8;  // giảm lắc góc vuông

double error = 0, PID_value = 0, Setpoint = 0;
double max_PID_value = 255 - motor_speed + 400;


//int flag = 0;
int memory = 0;
double memory2 = 0;
int memory3 = 0;
int memory4 = 0;
int k=0;

int LEFT = 0;
int RIGHT = 0;
//Khai báo PID
  PID myPID(&error, &PID_value, &Setpoint, Kp, Ki, Kd, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
//timer                                                                //P_ON_E (Proportional on Error) is the default behavior
  unsigned long time1;
  unsigned long time2;
  unsigned long time3;
  unsigned long deltatime3;
  
void setup()
{
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
//timer
  time1 = millis();
  time2 = millis();
  time3 = millis();
  
  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  delay(500);
  Serial.println("Started !!");
  delay(2000); 
  
  myPID.SetOutputLimits(-max_PID_value, max_PID_value);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(T);//40-30-20-10-12-12.5-13.50000-12.5-14-13-12.5-14-16-13.5-12-13
}

void loop()
{
  read_sensor_values();
  //denoise();
  Serial.print(error);
  // reverse  --nếu memory (errorfilter) ko hiệu quả thì reverse
  /*if ((error == 32)&&(time3 >= 10)&&(time3 <30))
   do{                            
      reverse();
      delay(30);
      read_sensor_values();
    } while (error == 32);
   else if (error == 32)
   {
    time3 = millis();
    error = memory2 ;
    }*/
  //Memory  
  if (error == 32) {
    deltatime3 = millis() - time3;
    if ( 20 <= deltatime3 < 100 ) error = 32;
    else {
      time3 = millis();
      error = memory2 ;
    } 
  }
  
  
  tao_memory();


// giảm tốc  
  if (error == 34) {                            
      motor_speed = 80;
      myPID.SetSampleTime(T1);
      //Serial.print(motor_speed);
      error = memory ;
      memory3 = 1; // nếu bằng 1 các hàm khác không được thay đổi motor_speed
      time1 = millis();
  }
    
    
    if ( ((unsigned long) (millis() - time1) > 1800)&&(memory3 !=0)&&(memory4 == 0)) //Phục hồi tự động nếu phục hồi sau rẽ ko chạy
    {
      memory3 = 0;
      motor_speed = initial_motor_speed;
      myPID.SetSampleTime(T);
    } else if ( ((unsigned long) (millis() - time1) > 200)&&(memory3 == 1)) //khử nhiễu
    {
     memory3 = 2; 
    }

    if ( ((unsigned long) (millis() - time2) > 200)&&(memory3 == 3)) //Phục hồi ngay sau rẽ
    {
      memory3 = 0;
      motor_speed = initial_motor_speed;
      myPID.SetSampleTime(T);
    }
    
    if ((error == -30)&&(memory3 == 2)) {               // Rẽ Trái 90*    Make left turn untill it detects straight path
    do {                            // Quay sang trái cho tới khi phát hiện ngay giữa line
      sharpLeftTurn();
      read_sensor_values();
    } while (error != 0);
     memory3 = 3; //giai đoạn sau rẽ
     time2 = millis();
  } else if ((error == 30)&&(memory3 == 2)){           // Rẽ Phải 90* Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
      do {                          // Quay sang trái cho tới khi phát hiện ngay giữa line
        sharpRightTurn();
        read_sensor_values();
      } while (error != 0);
     memory3 = 3;
     time2 = millis();
  }   
  else {
    error_filter();  //chống bắt line ngoài
    myPID.Compute();
    motor_control();                  // Điều chỉnh motor theo giá trị PID mới tính, cho xe chạy thẳng
    //Serial.print(PID_value);     
  }
}  

void read_sensor_values()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
  sensor[4] = digitalRead(sensor5);
//line trắng
if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
  error=6;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
  error=5;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==0))
  error=4;//3
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
  error=2;//1
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-2;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-4;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-5;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-6;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
    if (memory2 >= 0) error = 7;
    else error = 7;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4]==0)) // Turn robot left side
    error = -30;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4]==0)) // Turn robot left side
    error = -30;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4]==1)) // Turn robot right side
    error = 30;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4]==1)) // Turn robot right side
    error = 30;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) // Memory
    error = 32;
    
  //else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)&&(sensor[4]==1)) // Turn left side or stop
    //error = 33;
  else 
    error = 34;    //Memory
}
 

//line đen 
/*if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
  error=7;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=4.5;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==1))
  error=3;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
  error=1;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
  error=0;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
  error=-1;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=-3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=-4.5;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=-7;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4]==1)) // Turn robot left side
    error = -30;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4]==1)) // Turn robot left side
    error = -30;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4]==0)) // Turn robot right side
    error = 30;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4]==0)) // Turn robot right side
    error = 30;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) // Memory
    error = 32;
    
  //else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)&&(sensor[4]==0)) // Turn left side or stop
    //error = 103;
  else 
    error = 34;    //Memory
}*/




void tao_memory(){
 if (abs(error <= 2)){
    memory = error; //tạo memory
  } 
}

void error_filter(){
 //Chống bắt line viền  

    double key = error * memory2;
      if ((key >= 0) && (abs(error)<=10)){
        memory2 = error;
      }
      
      if (error == 32){
       //motor_speed = initial_motor_speed  ; 
       error = memory2 * 1.12 ;
      }
      else if ((abs(error)<=10) && (abs(error - memory2)>= 9))
      {
       //motor_speed = initial_motor_speed ; 
       error = memory2 * 1.12 ;
      }
 }
  //Serial.println(memory2);*/ 


void motor_control()
{
  // Calculating the effective motor speed:
  
  int left_motor_speed = motor_speed  + PID_value;   //
  int right_motor_speed = motor_speed - PID_value;  //
//  chuyển phần dư
  if (left_motor_speed > 255) {
    right_motor_speed = right_motor_speed - (left_motor_speed - 255);
  } else if (right_motor_speed > 255) {
      left_motor_speed = left_motor_speed - (right_motor_speed - 255);
  }
//  Đảo chiều 
  if (left_motor_speed < 0) {
    LEFT = 1;
    RIGHT = 0;
    left_motor_speed = 255 + left_motor_speed;
  } else if (right_motor_speed < 0) {
    RIGHT = 1;
    LEFT = 0;
    right_motor_speed = 255 + right_motor_speed;
  } else {
    LEFT = 0;
    RIGHT = 0;
  }

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);   //
  right_motor_speed = constrain(right_motor_speed, 0, 255); //


  analogWrite(motorInput1, left_motor_speed); //Left Motor Speed    //  
  analogWrite(motorInput3, right_motor_speed); //Right Motor Speed  //

  //following lines of code are to make the bot move forward
  forward();
}

void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  if (LEFT == 1) {
    digitalWrite(motorInput2, HIGH); //LOW = TIẾN
    digitalWrite(motorInput4, LOW);
  } else if (RIGHT == 1) {
    digitalWrite(motorInput2, LOW); //LOW = TIẾN
    digitalWrite(motorInput4, HIGH);
  } else {
     digitalWrite(motorInput2, LOW); //LOW = TIẾN
    digitalWrite(motorInput4, LOW);
  } 
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(motorInput1, 255 -90);
  digitalWrite(motorInput2, HIGH); //HIGH = LÙI
  analogWrite(motorInput3, 255 -90);
  digitalWrite(motorInput4, HIGH);
}

void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(motorInput1, banh_chinh);        //R Motor Speed
  digitalWrite(motorInput2, LOW);
  analogWrite(motorInput3, banh_phu-30);         //L Motor Speed
  digitalWrite(motorInput4, HIGH);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  double banh_chinh2 = banh_chinh ;
  analogWrite(motorInput1, banh_phu+35);        //Left Motor Speed
  digitalWrite(motorInput2, HIGH);
  analogWrite(motorInput3, banh_chinh2+5 );         //Right Motor Speed
  digitalWrite(motorInput4, LOW);
}
