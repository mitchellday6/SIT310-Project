
// defines pins numbers
const int echoPinLeft = 5;
const int trigPinLeft = 6;

const int echoPinFront = 11;
const int trigPinFront = 12;

const int echoPinRight = 2;
const int trigPinRight = 4;


//for ultrasonic sensor distance control
const int safeDistance = 15;  //distance at which robot should stop from object in cms
long duration;                
int distance;
bool isObject = false;


unsigned long speed = 10;   //speed of vehicle cm/sec
unsigned long angVel = 60;  //turning speed of vehicle deg/s
unsigned long endTime = 0;  //use to stop the current state/movment in the future

int motorright = 9;
int motorrightdir = 7;
int motorleft = 10;
int motorleftdir = 8;

// For serial receive.
const byte numChars = 11;     //number of characters in command string sent from server
char receivedChars[numChars]; // an array to store the received data
String received;              // The data as a string
boolean newData = false;

//for state control
enum STATES {FORWARD, BACKWARD, LEFT, RIGHT, STOP};   //list of possible states the vehicle could be in
STATES curState = STOP;                               //current state of the vehicle

void setup()
{
  pinMode(motorright, OUTPUT);
  pinMode(motorleft, OUTPUT);
  pinMode(motorrightdir, OUTPUT);
  pinMode(motorleftdir, OUTPUT);

  pinMode(trigPinLeft, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPinLeft, INPUT);   // Sets the echoPin as an Input
  pinMode(trigPinFront, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinFront, INPUT);  // Sets the echoPin as an Input
  pinMode(trigPinRight, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRight, INPUT);  // Sets the echoPin as an Input
  Serial.begin(9600);            // normal printing
}

void loop()
{
  int left = ultrasonic(echoPinLeft, trigPinLeft);
  int front = ultrasonic(echoPinFront, trigPinFront);
  int right = ultrasonic(echoPinRight, trigPinRight);
  
  if(front < safeDistance || left < safeDistance || right < safeDistance){
    if(!isObject){
       isObject = true;
       stop();
      }
  } else {
    if(isObject){
      isObject = false;
    }

    //check the current vehicle state and resume its movement
    switch(curState){
      case FORWARD:
        forward();
        break;
      case BACKWARD:
        backward();
        break;
      case LEFT:
        turnLeft();
        break;
      case RIGHT:
        turnRight();
        break;
      default: 
        stop();
        break;
    }
  }
  
  delay(100); // wait for 1/10th of a second.

  recvWithEndMarker();
  processCommand();
  checkTime();
}



int ultrasonic(int echoPin, int trigPin)
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}


//checks if the set endtime has been reached
//if so it stops the vehicle and turns its state back to stop
void checkTime(){
  if(curState == STOP) return;
  if(endTime < millis()){
    curState = STOP;
    stop();
  }
}

void processCommand()
{
  if (newData == true)
  {
    String instruction = received.substring(0, 5);
    String data = received.substring(6, 10);
    Serial.println(instruction+data);
    
    if (instruction == "MOVEF"){setForward(data.toInt());}
    if (instruction == "MOVEB"){setBackward(data.toInt());}
    if (instruction == "TURNL"){setLeft(data.toInt());}
    if (instruction == "TURNR"){setRight(data.toInt());}
    if(instruction == "STOPA"){
      curState == STOP;
      stop();
    }
    newData = false;
  }
}

void recvWithEndMarker()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (rc != endMarker)
    {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars)
      {
        ndx = numChars - 1;
      }
    }
    else
    {
      receivedChars[ndx] = '\0'; // terminate the string
      received = String(receivedChars);
      ndx = 0;
      newData = true;
    }
  }
}

//these methods set the vehicle movement, start the timer based on calculations and 
//set the current state of the vehicle

void setForward(int dist){
  unsigned long time = ((unsigned long)dist/speed)*1000;
  endTime = millis()+time;
  curState = FORWARD;
  forward();
}

void setBackward(int dist){
  unsigned long time = ((unsigned long)dist/speed)*1000;
  endTime = millis()+time;
  curState = BACKWARD;
  backward();
}

void setLeft(int deg){
  unsigned long time = ((unsigned long)deg/angVel)*1000;
  endTime = millis()+time;
  curState = LEFT;
  turnLeft();
}

void setRight(int deg){
  unsigned long time = ((unsigned long)deg/angVel)*1000;
  endTime = millis()+time;
  curState = RIGHT;
  turnRight();
}


// direction is controlled by the digital pin 7 and 8.
//  HIGH is backward, LOW is forward
//  Pins 9 and 10 control speed.

void forward()
{
  digitalWrite(motorrightdir, LOW);
  analogWrite(motorright, 180);
  digitalWrite(motorleftdir, LOW);
  analogWrite(motorleft, 180);
}

void backward()
{
  digitalWrite(motorrightdir, HIGH);
  analogWrite(motorright, 180);
  digitalWrite(motorleftdir, HIGH);
  analogWrite(motorleft, 180);
}
void turnLeft()
{
  digitalWrite(motorrightdir, LOW);
  analogWrite(motorright, 180);
  digitalWrite(motorleftdir, HIGH);
  analogWrite(motorleft, 180);
}

void turnRight()
{
  digitalWrite(motorrightdir, HIGH);
  analogWrite(motorright, 180);
  digitalWrite(motorleftdir, LOW);
  analogWrite(motorleft, 180);
}

void stop()
{
  analogWrite(motorright, 0);
  analogWrite(motorleft, 0);
}
