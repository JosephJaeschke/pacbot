#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "Wire.h"

#include "SharpDistSensor.h"

#include "PID.h"
#include "Flood.h"
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
#define INTERRUPT_PIN 4
#define MPU 0x68
#define SPEED 30
#define CIRC 3.14159265359*38.5 //from two years ago
#define TICKSPROT 174 //ticks per rotation (from two years ago)

bool debug = false;

// IR Sensor Initalization
const byte medianFilterWindowSize = 5;

SharpDistSensor front(IRF, medianFilterWindowSize);
SharpDistSensor fr(IRFR, medianFilterWindowSize);
SharpDistSensor fl(IRFL, medianFilterWindowSize);
SharpDistSensor br(IRBR, medianFilterWindowSize);
SharpDistSensor bl(IRBL, medianFilterWindowSize);

double frontSense = 0;
double leftSense = 0;
double rightSense = 0;

//encoder and wall PID Initalization
PID enc(3.8, 0.0, 0.0);
PID wall(4, 0, 0);
PID rotate(3,0,0);

//encoder count variables
volatile int leftCount=0, rightCount=0;
int prevR=0,prevL=0;

//Initalize the mpu to 0x68
MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float prevAngle;

// Create different states for robot

enum moveForwardStates {
  FORWARD_INITALIZE,  //Set motor speeds and start moving
  FORWARD,  //Move robot forward fixed distance or until it sees a wall
  FORWARD_CENTER, //After moving forward, if a wall is in front of the robot and it is too far backward, it moves to the center of the block
  FORWARD_FINALIZE //Stop all motors and reset values
  
};

enum moveForwardStates forwardState = FORWARD_INITALIZE;

enum rotateStates {
  ROTATE_INITALIZE, //Set initial angle
  ROTATING, //Rotate until desired angle is reached
  ROTATE_FINALIZE // Stop motors
};

enum rotateStates rotateState = ROTATE_INITALIZE;

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
double distance()
{
  double distR=abs((double)(rightCount-prevR)*CIRC/TICKSPROT);
  double distL=abs((double)(leftCount-prevL)*CIRC/TICKSPROT);
  return ((distR+distL)/2)/10;
}

// Move robot forward one block
void moveOne()
{
  switch (forwardState)
  {
    case FORWARD_INITALIZE:
        if(debug) Serial1.printf("Initalize\n");
        // Set motors to move forward
        digitalWrite(STBY,LOW);
        digitalWrite(AIN1,HIGH);
        digitalWrite(AIN2,LOW);
        digitalWrite(BIN1,HIGH);
        digitalWrite(BIN2,LOW);
        digitalWrite(STBY,HIGH);
        analogWrite(PWMA, SPEED);
        analogWrite(PWMB, SPEED);
        prevR=rightCount;
        prevL=leftCount;
        forwardState = FORWARD;
        
      break;

    // drive forward based on encoders and walls
    case FORWARD:
         if(debug) Serial1.printf("Forward\n");

        // Check if robot has traveled one blocks length, or is not in middle of a block based on front sensor
        if (distance() < 15.8 && frontSense > 45) {
          // Calculate encoder and wall error
          short encError=-(leftCount - prevL) + (rightCount - prevR);
          double encDiff=enc.compute(encError);

          double wallDiff = 0;
          
          // Check if there are valid walls to center on
          if (leftSense <45 && rightSense <45) {
            Serial1.printf("Found two walls\n");
            double wallError = leftSense - rightSense;
            wallDiff = wall.compute(wallError);
          }else{
            Serial1.printf("NO WALLS\n");
          }
          
          int adjust = SPEED - encDiff - wallDiff;
          adjust = constrain(adjust,0,100);
          analogWrite(PWMB, adjust);
        }
        //Traveled one block distance
        else if (distance() >= 15.8) {
          //Check if there is a wall in front of robot to center on
          if (frontSense < 100) {
            forwardState = FORWARD_CENTER;
          } else {
            forwardState = FORWARD_FINALIZE;
          }
        }
        // If the robot stopped because of a wall, or there is no wall
        else {
          forwardState = FORWARD_FINALIZE;
        }

      break;
      
    case FORWARD_CENTER:
      // While the robot is not in the center of box based on front wall, move forward
      if (frontSense > 45) {
        short encError=-(leftCount - prevL) + (rightCount - prevR);
        double encDiff=enc.compute(encError);

        double wallDiff = 0;
        // Check if there are valid walls to center on
        if (leftSense < 45 && rightSense < 45) {
          Serial1.printf("Found two walls\n");
          double wallError = leftSense - rightSense;
          wallDiff = wall.compute(wallError);
        }else{
          Serial1.printf("NO WALLS\n");
        }
    
        int adjust = SPEED - encDiff - wallDiff;
        
        adjust = constrain(adjust,0,100);
        analogWrite(PWMB, adjust);
      } else {
        forwardState = FORWARD_FINALIZE;
      }
      break;

    case FORWARD_FINALIZE:
      if(debug) Serial1.printf("Finalize\n");

      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,LOW);
      digitalWrite(BIN2,HIGH);
      analogWrite(PWMA, 10);
      analogWrite(PWMB, 10);
      delay(100);
      
      digitalWrite(STBY,LOW);
      prevR=rightCount;
      prevL=leftCount;

      // Set states of robot since moving forward is done
      robotState = IDLE;
      forwardState = FORWARD_INITALIZE;

      //Delay commented out since I dont know what its for
      //delay(50);
      
      break;
  }  
}

void turnCW()
{
  switch (rotateState) {
    case ROTATE_INITALIZE:
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,HIGH);
      digitalWrite(BIN2,LOW);
      analogWrite(PWMB,SPEED - 5);
      analogWrite(PWMA,SPEED - 5);
      digitalWrite(STBY,HIGH);

      prevR=rightCount;
      prevL=leftCount;
      
      rotateState = ROTATING;
      break;
      
    case ROTATING:
      if (prevL - leftCount > 88 && rightCount - prevR  > 88) {
        rotateState = ROTATE_FINALIZE;
      } else {
        short encError= (leftCount - prevL) - (prevR - rightCount);
        double encDiff= rotate.compute(encError);
    
        int adjust = SPEED - encDiff;
        
        adjust = constrain(adjust,0,100);
        analogWrite(PWMB, adjust);
      }
      break;

    case ROTATE_FINALIZE:
      digitalWrite(AIN1,HIGH);
      digitalWrite(AIN2,LOW);
      digitalWrite(BIN1,LOW);
      digitalWrite(BIN2,HIGH);
      digitalWrite(STBY,HIGH);
      analogWrite(PWMA, 10);
      analogWrite(PWMB, 10);
      delay(100);
      digitalWrite(STBY,LOW);

      prevR=rightCount;
      prevL=leftCount;
      
      rotateState = ROTATE_INITALIZE;
      robotState = IDLE;
      delay(100);
      break;
  }
}

void turnCCW()
{
  switch (rotateState) {
    case ROTATE_INITALIZE:
      digitalWrite(AIN1,HIGH);
      digitalWrite(AIN2,LOW);
      digitalWrite(BIN1,LOW);
      digitalWrite(BIN2,HIGH);
      analogWrite(PWMB,SPEED - 5);
      analogWrite(PWMA,SPEED - 5);
      digitalWrite(STBY,HIGH);

      prevR=rightCount;
      prevL=leftCount;
      
      rotateState = ROTATING;
      break;
      
    case ROTATING:
      if (leftCount - prevL > 88 && prevR - rightCount > 88) {
        rotateState = ROTATE_FINALIZE;
      } else {
        short encError= -(leftCount - prevL) + (prevR - rightCount);
        double encDiff= rotate.compute(encError);
    
        int adjust = SPEED - encDiff;
        
        adjust = constrain(adjust,0,100);
        analogWrite(PWMB, adjust);
      }
      break;
      
    case ROTATE_FINALIZE:
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,HIGH);
      digitalWrite(BIN2,LOW);
      digitalWrite(STBY,HIGH);
      analogWrite(PWMA, 10);
      analogWrite(PWMB, 10);
      delay(100);
      digitalWrite(STBY,LOW);

      prevR=rightCount;
      prevL=leftCount;
      
      rotateState = ROTATE_INITALIZE;
      robotState = IDLE;
      delay(100);
      break;
  }
}

void turnAround()
{
  switch (rotateState) {
    case ROTATE_INITALIZE:
      digitalWrite(AIN1,HIGH);
      digitalWrite(AIN2,LOW);
      digitalWrite(BIN1,LOW);
      digitalWrite(BIN2,HIGH);
      analogWrite(PWMB,SPEED - 5);
      analogWrite(PWMA,SPEED - 5);
      digitalWrite(STBY,HIGH);

      prevR=rightCount;
      prevL=leftCount;
      
      rotateState = ROTATING;
      break;
      
    case ROTATING:
      if (leftCount - prevL > 185 && prevR - rightCount > 185) {
        rotateState = ROTATE_FINALIZE;
      } else {
        short encError= -(leftCount - prevL) + (prevR - rightCount);
        double encDiff= rotate.compute(encError);
    
        int adjust = SPEED - encDiff;
        
        adjust = constrain(adjust,0,100);
        analogWrite(PWMB, adjust);
      }
      break;
      
    case ROTATE_FINALIZE:
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,HIGH);
      digitalWrite(BIN2,LOW);
      digitalWrite(STBY,HIGH);
      analogWrite(PWMA, 10);
      analogWrite(PWMB, 10);
      delay(100);
      digitalWrite(STBY,LOW);

      prevR=rightCount;
      prevL=leftCount;
      
      rotateState = ROTATE_INITALIZE;
      robotState = IDLE;
      delay(100);
      break;
  }
}

void halt()
{
	digitalWrite(STBY,LOW);
}

float frontBuffer [20];
int frontBuffLen = 20;
float variance = 0;
void sense()
{
  /*
  // Calculate Variance of front sensor to deal with random vals
  float sum = 0;
  for (int i = 0; i < frontBuffLen - 1; i++) {
    frontBuffer[i+1] = frontBuffer[i];
    sum += frontBuffer[i];
  }
  frontBuffer[0] = front.DEMA;
  sum += front.DEMA;

  float mean = sum / frontBuffLen;

  variance = 0;
  for (int i = 0; i < frontBuffLen; i++) {
    variance += pow((frontBuffer[i] - mean), 2);
  }
  variance = variance / frontBuffLen;

  if (variance < 1.0f) {
    frontSense = front.DEMA;
  } else {
    frontSense = 0;
  }
  */
  frontSense = front.getDist();
  leftSense = (fl.getDist() + bl.getDist())/2.0f;
  rightSense = (fr.getDist() + br.getDist())/2.0f;
}

void setSpace(short row,short col)
{
  for(int i=0;i<5;i++)
  {
    sense();
  }
  bool fwall=frontSense<55 ? true:false;
  bool rwall=leftSense<45 ? true:false;
  bool lwall=rightSense<45 ? true:false;
  if(facing=='n')
  {
    grid[row][col].west=lwall;
    grid[row][col].north=fwall;
    grid[row][col].east=rwall;
  }
  else if(facing=='e')
  {
    grid[row][col].north=lwall;
    grid[row][col].east=fwall;
    grid[row][col].south=rwall;
  }
  else if(facing=='s')
  {
    grid[row][col].east=lwall;
    grid[row][col].south=fwall;
    grid[row][col].west=rwall;
  }
  else //facing=='w'
  {
    grid[row][col].south=lwall;
    grid[row][col].west=fwall;
    grid[row][col].north=rwall;
  }
}




volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//unsigned long prevTime = 0;
void readMPU6050() {
  if (mpuInterrupt && mpu.getFIFOCount() > packetSize) {
    
    //Serial1.printf("Elapsed Time: %lu\n", millis() - prevTime);
   // prevTime = millis();
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      ypr[0] = ypr[0] * 180/M_PI + 180;

    }
  }
}

void setup()
{
  Serial.begin(57600);
  Serial1.begin(115200);
  
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
  front.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  fr.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  fl.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  br.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  bl.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);

  attachInterrupt(digitalPinToInterrupt(LEOA),leftEncoderEvent,CHANGE);
  attachInterrupt(digitalPinToInterrupt(REOA),rightEncoderEvent,CHANGE);
  
  Wire.setSDA(30);
  Wire.setSCL(29);  
  Wire.begin();
  Wire.setClock(400000);
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  //Serial1.println(F("Testing device connections..."));
  //Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
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
  
  digitalWrite(13,HIGH);
  analogWrite(PWMA,SPEED);
  analogWrite(PWMB,SPEED);
  initializeGrid();
  curr=grid[0][0];
  delay(200);
  
}

float printTime = 0;
void loop()
{
  
  if (Serial1.available() > 0)
  {
    char xbeeIn = (char)Serial1.read();
    Serial.write(xbeeIn);
    switch (xbeeIn) 
    {
      case '1':
        Serial1.write("Moving Forward\n");
        robotState = MOVE_FORWARD;
        break;
      case '2':
        Serial1.write("Turning cw\n");
        robotState = ROTATE_CW_FORWARD;
        break;
      case '3':
        Serial1.write("Turning ccw\n");
        robotState = ROTATE_CCW_FORWARD;
        break;
      case'4':
        Serial1.write("Rotate 180 and forward\n");
        robotState = TURN_FORWARD;
        break;
      case'5':
        Serial1.printf("CurrAngle: %f\nPrevAngle: %f\nModDifference: %d\n\n", ypr[0], prevAngle, (int)(prevAngle + (360 - ypr[0]) + 5) % 360);
        Serial1.printf("Front: %f\nLeft: %f\nRight: %f\n", frontSense, leftSense, rightSense);
        Serial1.printf("leftEnc: %d\nrightEnc%d\n\n", leftCount, rightCount);
        break;
      case'6':
        digitalWrite(AIN1,HIGH);
        digitalWrite(AIN2,LOW);
        digitalWrite(BIN1,HIGH);
        digitalWrite(BIN2,LOW);
        digitalWrite(STBY,HIGH);
        digitalWrite(STBY,HIGH);
        analogWrite(PWMA, 5);
        analogWrite(PWMB, 5);
        break;
      case'7':
        digitalWrite(STBY,LOW);
        analogWrite(PWMA, 5);
        analogWrite(PWMB, 5);
        break;
    } 
  }
  floodFill();
  switch (robotState)
  {
    case MOVE_FORWARD:
      moveOne();
      break;
    case ROTATE_CW_FORWARD:
      turnCW();
      break;
    case ROTATE_CCW_FORWARD:
      turnCCW();
      break;
    case TURN_FORWARD:
      turnAround();
    case IDLE:
      break;
  }
  
  // Always update the ir sensors every cycle
  sense();

  // Read MPU6050 Sensor
  readMPU6050();
  
  //Debug output
  //Serial1.printf("CurrAngle: %f\nPrevAngle: %f\nModDifference: %d\n\n", ypr[0], prevAngle, (int)(prevAngle + (360 - ypr[0]) + 5) % 360);
  //Serial1.printf("Front: %f\nLeft: %f\nRight: %f\n", frontSense, leftSense, rightSense);
  //Serial1.printf("VARIANCE: %f\n\n", variance);
  delay(20);
}
