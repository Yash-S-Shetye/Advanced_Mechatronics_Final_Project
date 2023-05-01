#include <Servo.h>
#include <SoftwareSerial.h>
#define black 1
#define white 0
#define obsdistance 20
#define nighty 1000
#define oneeighty 2000

bool finish=false;
bool localfinish=false;
int c_intersection=0;
int parkinglot[8]={0,0,0,0,0,0,0,0};
int parkidx=0;
int obstacles_no=0;
int c_tw=0;

//Creating a robot class containing all the functions will be used
class Robot {
  private:
  //Defining required variables
  const int leftwheel=2;
  const int rightwheel=3;
  const int ultrasonic=11;
  const int TxPin = 12;
  const int IR_ML=4;
  const int IR_MR=8;
  const int led=7;
  Servo servoLeft, servoRight;
  
  
  public:
  void INIT();
  SoftwareSerial mySerial = SoftwareSerial(255, TxPin);
  //Declaring all the required functions
  bool isobstacle();
  void drive(char i);
  void lcd_display(char disp);
  bool linefollowing();
  void test();
};

//Initialization
void Robot::INIT(){
  servoLeft.attach(leftwheel);
  servoRight.attach(rightwheel);
  mySerial.begin(9600);
  delay(100);
  mySerial.write(12); // Clear
  mySerial.write(17); // Turn backlight on
  delay(5); // Required delay
  mySerial.print("Initialized!!!");delay(2000);
  pinMode(led,OUTPUT);
}

//Defining getDistance function
bool Robot::isobstacle() {
  long duration;
  int distance;
  pinMode(ultrasonic,OUTPUT);
  digitalWrite(ultrasonic, LOW); // set the pin to low
  delayMicroseconds(2); // wait for 2 microseconds
  digitalWrite(ultrasonic, HIGH); // set the pin to high to trigger the sensor
  delayMicroseconds(10); // wait for 10 microseconds
  digitalWrite(ultrasonic, LOW); // set the pin to low again to switch to echo mode

  pinMode(ultrasonic,INPUT);
  duration = pulseIn(ultrasonic, HIGH); // measure the duration of the sound wave travel
  distance = duration * 0.034 / 2; // calculate the distance in cm
  
  
  if (distance<obsdistance){
    digitalWrite(led,HIGH);delay(50);
    digitalWrite(led,LOW);delay(50);
    return true;}
  else{
    //digitalWrite(led,LOW);delay(10);
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
    // Stop
    drive('s');return false;
  }
}
