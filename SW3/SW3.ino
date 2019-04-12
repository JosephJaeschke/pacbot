//#include "MPU6050_6Axis_MotionApps20.h"
//#include "I2Cdev.h"
//#include "helper_3dmath.h"
//#include "Wire.h"

#include "PID.h"
#include "Sensor.h"
#include "SoftwareSerial.h"

#define STBY 20
#define AIN2 22
#define AIN1 23
#define BIN2 11
#define BIN1 12
#define LEOB 6
#define LEOA 5
#define REOA 8
#define REOB 7
#define PWMA 10
#define PWMB 9
#define IRBL A7
#define IRF A1
#define IRBR A3
#define IRFR A2
#define IRFL A0
#define INTERRUPT_PIN 1
#define MPU 0x68
#define SPEED 30
#define CIRC 3.14159265359*38.5 //from two years ago
#define TICKSPROT 174 //ticks per rotation (from two years ago)

int16_t AcX=0,AcY=0,AcZ=0,GyX=0,GyY=0,GyZ=0,temperature=0;

// IR Sensor Initalization
Sensor front(IRF);
Sensor fr(IRFR);
Sensor fl(IRFL);
Sensor br(IRBR);
Sensor bl(IRBL);

float frontSense = 0;
float leftSense = 0;
float rightSense = 0;

//encoder and wall PID Initalization
PID enc(3.8,0.0,0.0);
PID wall(.5, 0, 0);

//encoder count variables
volatile int leftCount=0, rightCount=0;
int prevR=0,prevL=0;

////Initalize the mpu to 0x68
//MPU6050 mpu;
//
//bool blinkState = false;
//
//// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//
//
//// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//float euler[3];         // [psi, theta, phi]    Euler angle container

// Create different states for robot

enum states {
  IDLE, //Waits for its next movement state
  MOVE_FORWARD, //Move forward one block and switch back to idle
  ROTATE_CW,  //Rotate 90 degrees Clockwise
  ROTATE_CCW  //Rotate 90 degrees Counter Clock Wise
};

enum states robotState = IDLE;

enum moveForwardStates {
  INITALIZE,  //Set motor speeds and start moving
  FORWARD,  //Move robot forward fixed distance or until it sees a wall
  FORWARD_CENTER, //After moving forward, if a wall is in front of the robot and it is too far backward, it moves to the center of the block
  FINALIZE //Stop all motors and reset values
  
};

enum moveForwardStates forwardState = INITALIZE;

//Interrupt events for encoders
void leftEncoderEvent() 
{
  if(digitalRead(LEOA)==HIGH)
  {
    if(digitalRead(LEOB)==LOW)
    {
      leftCount++;
    }
    else
    {
      leftCount--;
    }
  } 
  else
  {
    if(digitalRead(LEOB)==LOW)
    {
      leftCount--;
    }
    else 
    {
      leftCount++;
    }
  }
}

void rightEncoderEvent() 
{
  if(digitalRead(REOA)==HIGH)
  {
    if(digitalRead(REOB)==LOW)
    {
      rightCount++;
    }
    else
    {
      rightCount--;
    }
  } 
  else
  {
    if(digitalRead(REOB)==LOW)
    {
      rightCount--;
    }
    else
    {
      rightCount++;
    }
  }

}

// Distance Converter from encoders to cm
float distance()
{
  float distR=abs((float)(rightCount-prevR)*CIRC/TICKSPROT);
  float distL=abs((float)(leftCount-prevL)*CIRC/TICKSPROT);
  return ((distR+distL)/2)/10;
}

// Move robot forward one block
void moveOne()
{
  switch (forwardState)
  {
    case INITALIZE:
        // Set motors to move forward
        digitalWrite(STBY,LOW);
        digitalWrite(AIN1,HIGH);
        digitalWrite(AIN2,LOW);
        digitalWrite(BIN1,HIGH);
        digitalWrite(BIN2,LOW);
        digitalWrite(STBY,HIGH);

        forwardState = FORWARD;
        
      break;

    // drive forward based on encoders and walls
    case FORWARD:
        // Check if robot has traveled one blocks length, or is not in middle of a block based on front sensor
        if (distance() < 15.8 || frontSense < 260) {
          // Calculate encoder and wall error
          short encError=-leftCount+rightCount;
          float encDiff=enc.compute(encError);

          float wallDiff = 0;
          
          // Check if there are valid walls to center on
          if (leftSense > 170 && rightSense > 170) {
            float wallError = -leftSense + rightSense;
            wallDiff = wall.compute(wallError);
          }
      
          int adjust = SPEED - encDiff - wallDiff;
          adjust = constrain(adjust,0,100);
          analogWrite(PWMB, adjust);
        }
        //Traveled one block distance
        else if (distance() >= 15.8) {
          //Check if there is a wall in front of robot to center on
          if (frontSense > 140) {
            forwardState = FORWARD_CENTER;
          } else {
            forwardState = FINALIZE;
          }
        }
        // If the robot stopped because of a wall, or there is no wall
        else {
          forwardState = FINALIZE;
        }

      break;
      
    case FORWARD_CENTER:
      // While the robot is not in the center of box based on front wall, move forward
      if (frontSense < 260) {
        short encError=-leftCount+rightCount;
        float encDiff=enc.compute(encError);

        float wallDiff = 0;
        // Check if there are valid walls to center on
        if (leftSense > 170 && rightSense > 170) {
          float wallError = -leftSense + rightSense;
          wallDiff = wall.compute(wallError);
        }
    
        int adjust = SPEED - encDiff - wallDiff;
        
        adjust = constrain(adjust,0,100);
        analogWrite(PWMB, adjust);
      } else {
        forwardState = FINALIZE;
      }
      break;

    case FINALIZE:
      digitalWrite(STBY,LOW);
      prevR=rightCount;
      prevL=leftCount;

      // Set states of robot since moving forward is done
      robotState = IDLE;
      forwardState = INITALIZE;

      //Delay commented out since I dont know what its for
      //delay(50);
      
      break;
  }  
}


void turnCW()
{
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  analogWrite(PWMB,SPEED);
  digitalWrite(STBY,HIGH);
  rightCount=0;
  leftCount=0;
  while(rightCount < 88 && leftCount > -88)//double check
  {
    Serial1.printf("Left Enc: %d Right Enc: %d \n", leftCount, rightCount);
  }
  digitalWrite(STBY,LOW);
  delay(100);
  rightCount=0;
  leftCount=0;
  prevR=0;
  prevL=0;
  delay(100);
}

void turnCCW()
{
	digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
  analogWrite(PWMB,SPEED);
  digitalWrite(STBY,HIGH);
  rightCount=0;
  leftCount=0;
  while(rightCount > -93 &&leftCount < 93)//double check
  {
    Serial1.printf("Left Enc: %d Right Enc: %d \n", leftCount, rightCount);
  }
  digitalWrite(STBY,LOW);
  delay(100);
  rightCount=0;
  leftCount=0;
  prevR=0;
  prevL=0;
  delay(100);
}

void halt()
{
	digitalWrite(STBY,LOW);
}

void sense()
{
  for(int i=0;i<5;i++)
  {
    front.filter(analogRead(front.pin));
    fr.filter(analogRead(fr.pin));
    fl.filter(analogRead(fl.pin));
    br.filter(analogRead(br.pin));
    bl.filter(analogRead(bl.pin));
  }
  frontSense = front.DEMA;
  leftSense = (fl.DEMA+bl.DEMA)/2.0f;
  rightSense = (fr.DEMA + br.DEMA)/2.0f;
}

/*
void setSpace(short row,short col)
{
  sense();
  bool fwall=false;
  bool rwall=false;
  bool lwall=false;
  if(front.DEMA>front.thresholdd)
  {
    fwall=true;
  }
  if(left.DEMA>left.thresholdd)
  {
    lwall=true;
  }
  if(right.DEMA>right.thresholdd)
  {
    rwall=true;
  }
  if(fwall&&grid[row][col].visited==0)
  {
    if(facing=='u')
    {
      grid[row][col].up=1;
    }
    else if(facing=='r')
    {
      grid[row][col].right=1;
    }
    else if(facing=='d')
    {
      grid[row][col].down=1;
    }
    else
    {
      grid[row][col].left=1;
    }
  }
  if(lwall&&grid[row][col].visited==0)
  {
    if(facing=='u')
    {
      grid[row][col].left=1;
    }
    else if(facing=='r')
    {
      grid[row][col].up=1;   
    }
    else if(facing=='d')
    {
      grid[row][col].right=1;
    }
    else
    {
      grid[row][col].down=1;
    }
  }
  if(rwall&&grid[row][col].visited==0)
  {
    if(facing=='u')
    {
      grid[row][col].right=1;
    }
    else if(facing=='r')
    {
      grid[row][col].down=1;
    }
    else if(facing=='d')
    {
      grid[row][col].left=1;
    }
    else
    {
      grid[row][col].up=1;
    }
    delay(1000);
  }
  use_enc=true;
  if(rwall&&lwall)
  {
   short error=left.DEMA-right.DEMA;
   float diff=irPID.compute(error);
   int adjust=SPEED-diff;
   adjust=constrain(adjust,0,255);
   analogWrite(PWMB,adjust);
   use_enc=false;
  }
}
*/


//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//    mpuInterrupt = true;
//}
//
//void readMPU6050() {
//  if (mpuInterrupt && mpu.getFIFOCount() < packetSize) {
//
//    // reset interrupt flag and get INT_STATUS byte
//    mpuInterrupt = false;
//    mpuIntStatus = mpu.getIntStatus();
//
//    // get current FIFO count
//    fifoCount = mpu.getFIFOCount();
//
//    // check for overflow (this should never happen unless our code is too inefficient)
//    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
//        // reset so we can continue cleanly
//        mpu.resetFIFO();
//        fifoCount = mpu.getFIFOCount();
//        Serial.println(F("FIFO overflow!"));
//
//    // otherwise, check for DMP data ready interrupt (this should happen frequently)
//    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
//      // wait for correct available data length, should be a VERY short wait
//      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//      // read a packet from FIFO
//      mpu.getFIFOBytes(fifoBuffer, packetSize);
//      
//      // track FIFO count here in case there is > 1 packet available
//      // (this lets us immediately read more without waiting for an interrupt)
//      fifoCount -= packetSize;
//      // display Euler angles in degrees
//      mpu.dmpGetQuaternion(&q, fifoBuffer);
//      mpu.dmpGetEuler(euler, &q);
//      Serial.print("euler\t");
//      Serial.print(euler[0] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(euler[1] * 180/M_PI);
//      Serial.print("\t");
//      Serial.println(euler[2] * 180/M_PI);
//    }
//  }
//}

void setup()
{
  pinMode(13,OUTPUT);
  pinMode(STBY,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(LEOA,INPUT);
  pinMode(LEOB,INPUT);
  pinMode(REOA,INPUT);
  pinMode(REOB,INPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(IRBL,INPUT);
  pinMode(IRF,INPUT);
  pinMode(IRBR,INPUT);
  pinMode(IRFR,INPUT);
  pinMode(IRFL,INPUT);
  attachInterrupt(digitalPinToInterrupt(LEOA),leftEncoderEvent,CHANGE);
  attachInterrupt(digitalPinToInterrupt(REOA),rightEncoderEvent,CHANGE);
  Serial.begin(9600);
  Serial1.begin(115200);
  /*
  Wire.begin();
  Wire.setSDA(30);
  Wire.setSCL(29);
  Wire.setClock(400000);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial1.println(F("Testing device connections..."));
  Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
        
  mpu.setDMPEnabled(true);
  
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  */
  digitalWrite(13,HIGH);
  analogWrite(PWMA,SPEED);
  analogWrite(PWMB,SPEED);
  delay(200);
}

void loop()
{
  /*if (Serial1.available() > 0)
  {
    char xbeeIn = (char)Serial1.read();
    Serial.write(xbeeIn);
    switch (xbeeIn) 
    {
      case '1':
        Serial.write("Moving Forward");
        robotState = MOVE_FORWARD;
        break;
      case '2':
        Serial.write("Turning cw");
        robotState = ROTATE_CW;
        break;
      case '3':
        Serial.write("Turning ccw");
        robotState = ROTATE_CCW;
        break;
    } 
  }
  
  switch (robotState)
  {
    case MOVE_FORWARD:
      moveOne();
      break;
    case ROTATE_CW:
      turnCW();
      break;
    case ROTATE_CCW:
      turnCCW();
      break;
    case IDLE:
      break;
  }
  */
  // Always update the ir sensors every cycle
  sense();

  // Read MPU6050 Sensor
  //readMPU6050();
  //Debug output
  
  Serial.printf("Front: %f\nLeft: %f\nRight: %f\n", frontSense, leftSense, rightSense);
  delay(50);
  
}
