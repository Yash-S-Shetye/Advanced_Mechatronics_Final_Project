#include <Servo.h>
#include <SoftwareSerial.h>
#define black 1
#define white 0
#define obsdistance 10
#define nighty 1000
#define oneeighty 2000
const int led = 9;

bool finish=false;                      //flag variable inidicates if the job is done
bool localfinish=false;
int c_good=0;
int c_bad=0;                          //indicates which parking lot the robot is going trough
//Create a robot class containing all the functions will be used
class Robot {
  private:
  // Defining required variables: pin configuration
  const int leftwheel=2;
  const int rightwheel=3;
  const int ultrasonic=11;
  const int TxPin = 12;
  const int IR_ML=4;
  const int IR_MR=8;
  const int trig=6;
  const int echo=7;
  SoftwareSerial mySerial = SoftwareSerial(255, TxPin);
  
  
  public:
  //Declaring all the required functions
  Servo servoLeft, servoRight;
  void INIT();
  bool isobjectl();
  bool isobjectr();
  void drive(char i);
  void lcd_display(char disp);
  bool linefollowing();
  void goforaleftturn();
  void goforarightturn();
  void test();
};

//Initialization: initialize the motors and lcd
void Robot::INIT(){
  servoLeft.attach(leftwheel);
  servoRight.attach(rightwheel);
  mySerial.begin(9600);
  delay(100);
  mySerial.write(12); // Clear
  mySerial.write(17); // Turn backlight on
  delay(5); // Required delay
  mySerial.print("Initialized!!!");delay(2000);mySerial.write(12);
  pinMode(led,OUTPUT);
}

//Defining getDistance function
bool Robot::isobjectl() {
  long duration;
  int distance;
  pinMode(ultrasonic,OUTPUT);
  digitalWrite(ultrasonic, LOW);    // set the pin to low
  delayMicroseconds(2);             // wait for 2 microseconds
  digitalWrite(ultrasonic, HIGH);   // set the pin to high to trigger the sensor
  delayMicroseconds(10);            // wait for 10 microseconds
  digitalWrite(ultrasonic, LOW);    // set the pin to low again to switch to echo mode

  pinMode(ultrasonic,INPUT);
  duration = pulseIn(ultrasonic, HIGH);   // measure the duration of the sound wave travel
  distance = duration * 0.034 / 2;        // calculate the distance in cm
  
  if (distance<obsdistance){
    return true;}
  else{
    return false;}
}

bool Robot::isobjectr() {
  long duration;
  int distance;
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  digitalWrite(trig, LOW);    // set the pin to low
  delayMicroseconds(2);             // wait for 2 microseconds
  digitalWrite(trig, HIGH);   // set the pin to high to trigger the sensor
  delayMicroseconds(10);            // wait for 10 microseconds
  digitalWrite(trig, LOW);    // set the pin to low again to switch to echo mode

  duration = pulseIn(echo, HIGH);   // measure the duration of the sound wave travel
  distance = duration * 0.034 / 2;        // calculate the distance in cm
  
  
  if (distance<obsdistance){
    return true;}
  else{
    return false;}
}

//Defining drive function
void Robot::drive(char i) {
  switch(i){
  // f, b, l, r, s means forward, backward, left, right, and stop
    case 'f':servoLeft.write(1550);servoRight.write(1440);delay(20);break;
    case 'b':servoLeft.write(1450);servoRight.write(1550);delay(20);break;
    case 'l':servoLeft.write(1450);servoRight.write(1450);delay(20);break;
    case 'r':servoLeft.write(1550);servoRight.write(1550);delay(20);break;
    case 's':servoLeft.write(1500);servoRight.write(1500);delay(20);break;
    default:Serial.println("Unclear command for motors");break;
  }
}

//run this method continuesly to follow the line until the robot meets an intersection
bool Robot::linefollowing(){
  int SL = digitalRead(IR_ML);
  int SR = digitalRead(IR_MR);
  if (SL == white && SR == white) {
    //forward
    drive('f');return true;
  }
  else if (SL == black && SR== white) {
    //turn left
    drive('l');return true;
  }
  else if (SL == white &&  SR == black) {
    // Turn right
    drive('r');return true;
  }
  else {
    // Stop if theres an intersection
    drive('s');return false;
  }
}

void Robot::goforaleftturn(){
  while(linefollowing()){}
  drive('f');delay(500);
  drive('l');delay(nighty);
}

void Robot::goforarightturn(){
  while(linefollowing()){}
  drive('f');delay(500);
  drive('r');delay(nighty);
}

//Defining lcd display function
void Robot::lcd_display(char disp) {
  mySerial.write(12); // Clear
  mySerial.write(17); // turn on the backlight
  delay(5); // Required delay
  
  switch(disp) {
    case 'n': mySerial.print("Not Defective");mySerial.write(12);break;           // n - Non defective widget detected
    case 'd': mySerial.print("Defective");mySerial.write(12);break;         // d - Defective widget detected
    case 'e': mySerial.print("Not Defective");mySerial.write(13);mySerial.print(c_good); // print total number of non defective widgets
              mySerial.write(12);mySerial.write("Defective");mySerial.write(13);mySerial.print(c_bad); // print total number of defective widgets
              mySerial.write(12);break;
    default:Serial.println("Unclear command for display");break;
  }
}

//for test purpose
void Robot::test(){
  servoLeft.write(1500);servoRight.write(1500);
}

//create robot object as a global variable
Robot rob;

void setup() {
  //initialize objects of Robot class
  Serial.begin(9600);
  rob.INIT();
}

void loop() {
  if(!finish){
    rob.drive('s');delay(1000);
    unsigned long start_time,end_time;
    char direction=' ';   // or 'r'
    char good=' ';        // or 'f'
    for(int corner=1;corner<=3;corner++){
      while(rob.linefollowing()){}
      rob.drive('s');
      digitalWrite(led, HIGH);
      delay(500);
      digitalWrite(led, LOW);
      delay(500);

      // send_time signal to rpi, check triangle, then move on
      Serial.print('t');
      while(true){
        if(Serial.available()){
          direction=Serial.read();
          if(direction=='l'||direction=='r')break;
        }
      }

      if(direction=='l'){                     //triangle point to the left
        rob.drive('f');delay(500);
        rob.drive('l');delay(nighty);
        rob.goforarightturn();
        while(rob.linefollowing()){
          if(rob.isobjectl()){
            rob.drive('l');delay(nighty);
            rob.drive('b');delay(500);
            rob.drive('s');
             Serial.print('a');
             start_time=millis();
             while(true){
               end_time=millis();
               if(end_time-start_time>2000)break;
               if(Serial.available()){
                 good=Serial.read();
                 if(good=='t'){rob.lcd_display('n');c_good++;}
                 if(good=='f'){rob.lcd_display('d');c_bad++;}
               }
               
             }
            break;
          }
        }
        rob.drive('f');delay(500);
        rob.drive('l');delay(nighty);
        rob.goforaleftturn();
        while(rob.linefollowing()){}
        rob.drive('f');delay(500);
        rob.goforaleftturn();
        while(rob.linefollowing()){
          if(rob.isobjectr()){
            rob.drive('r');delay(nighty);
            rob.drive('b');delay(500);
            rob.drive('s');
            Serial.print('a');
             start_time=millis();
             while(true){
               end_time=millis();
               if(end_time-start_time>2000)break;
               if(Serial.available()){
                 good=Serial.read();
                 if(good=='n'){rob.lcd_display('n');c_good++;}
                 if(good=='d'){rob.lcd_display('d');c_bad++;}
               }
               
             }
            break;
          }
        }
        rob.drive('f');delay(500);rob.drive('l');delay(nighty);
        rob.goforaleftturn();
        rob.goforarightturn();
      }
      else{                                 //triangle point to the right
        rob.drive('f');delay(500);
        rob.drive('r');delay(nighty);
        rob.goforaleftturn();
        while(rob.linefollowing()){
          if(rob.isobjectr()){
            rob.lcd_display('n');
            rob.drive('r');delay(nighty);
            rob.drive('b');delay(500);
            rob.drive('s');
            Serial.print('a');
             start_time=millis();
             while(true){
               end_time=millis();
               if(end_time-start_time>2000)break;
               if(Serial.available()){
                 good=Serial.read();
                 if(good=='n'){rob.lcd_display('n');c_good++;}
                 if(good=='d'){rob.lcd_display('d');c_bad++;}
               }
             }
            break;
          }
        }
        rob.drive('f');delay(500);
        rob.drive('r');delay(nighty);
        rob.goforarightturn();
        while(rob.linefollowing()){}
        rob.drive('f');delay(500);
        rob.goforarightturn();
        while(rob.linefollowing()){
          if(rob.isobjectl()){
            rob.drive('l');delay(nighty);
            rob.drive('b');delay(500);
            rob.drive('s');
            Serial.print('a');
             start_time=millis();
             while(true){
               end_time=millis();
               if(end_time-start_time>2000)break;
               if(Serial.available()){
                 good=Serial.read();
                 if(good=='n'){rob.lcd_display('n');c_good++;}
                 if(good=='d'){rob.lcd_display('d');c_bad++;}
               }
               
             }
            break;
          }
        }
        rob.drive('f');delay(500);
        rob.drive('r');delay(nighty);
        rob.goforarightturn();
        rob.goforaleftturn();
      }
    }

    // to the end_time
    char dir=' ';
    char flag=' ';
    start_time=millis();
    end_time=millis();
    while(end_time-start_time<3000){
      rob.linefollowing();
      end_time=millis();
    }
    rob.drive('s');
     Serial.print('t');
     while(true){
       if(Serial.available()){
        dir=Serial.read();
        if(dir=='l'||dir=='r'){
         break;
         }
       }
       
     }
    
    Serial.print('f');
    if(dir=='r'){
      while(true){
        rob.servoLeft.write(1510);rob.servoRight.write(1510);delay(20);
        if(Serial.available()){flag=Serial.read();}
        if(flag=='g')break;
      }
    }
    else{
      while(true){
        rob.drive('l');
        rob.servoLeft.write(1495);rob.servoRight.write(1495);delay(20);
        if(Serial.available()){flag=Serial.read();}
        if(flag=='g')break;
      }
    }
    rob.drive('f');delay(3000);
    rob.drive('s');delay(100);
    rob.lcd_display('e');
    finish=true;
  }
}
