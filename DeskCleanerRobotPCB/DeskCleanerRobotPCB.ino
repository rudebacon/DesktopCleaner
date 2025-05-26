  //CONTROL OF MOTORS: analogWrite range is 0-255 but speed depends on pin1
  // digitalWrite(AIN1_PIN, LOW);  // will spin forward direction full speed
  // analogWrite(AIN2_PIN, 255);

  // digitalWrite(AIN1_PIN, LOW);  // will stop (in forward direction)
  // analogWrite(AIN2_PIN, 0);

  // digitalWrite(AIN1_PIN, LOW);  // will spin forward direction slowly
  // analogWrite(AIN2_PIN, 50);

  // digitalWrite(AIN1_PIN, HIGH);  // will spin reverse direction full speed
  // analogWrite(AIN2_PIN, 0);

  // digitalWrite(AIN1_PIN, HIGH);  // will spin reverse direction slowly speed
  // analogWrite(AIN2_PIN, 205);

  // digitalWrite(AIN1_PIN, HIGH);  // will stop (in reverse direction)
  // analogWrite(AIN2_PIN, 255);

#include <ESP32Servo.h>




#include <Wire.h>
#include <Adafruit_VCNL4040.h>  // Include the VCNL4040 library

const int escPin = 39;


// const int escPin = 39;  // IO39 for ESC signal
Servo esc;



byte busStatus;
bool i2cDeviceFound = false; // Flag to indicate if an I2C device is detected

#define LED_PIN 8 // GPIO 8 for LED

#define MODE_PIN 40  // GPIO 40 connected to MODE
#define AIN1_PIN 41  // GPIO 41 connected to AIN1 (PHASE)
#define AIN2_PIN 42  // GPIO 42 connected to AIN2 (ENABLE)
#define BIN1_PIN 45   // GPIO 45 connected to BIN1 (PHASE)
#define BIN2_PIN 46   // GPIO 46 connected to BIN2 (ENABLE)

#define PCA9548A_ADDR 0x70  // I2C address of the PCA9548A multiplexer

Adafruit_VCNL4040 vcnl;  // Create an instance of the VCNL4040 sensor


// Function to select the channel on the PCA9548A expander
void selectPCA9548AChannel(uint8_t channel) {
  if (channel > 7) return; // Ensure valid channel (0-7)
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);  // Activate the desired channel
  Wire.endTransmission();
}

// Converts microseconds to duty cycle for 50Hz and 16-bit
uint32_t usToDuty(uint16_t microseconds) {
  return (microseconds * 65535UL) / 20000UL;
}


void setup() {
  Serial.begin(115200);

  // Configure LED pin as output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Ensure LED is off initially

  // Configure motor pins as outputs //UNCOMMENT MOTOR STUFF LATER
  pinMode(MODE_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  // Set MODE to PHASE/ENABLE mode
  digitalWrite(MODE_PIN, LOW);
  // Set motor to forward direction
  digitalWrite(AIN1_PIN, HIGH);
  // Set motor to no power 
  analogWrite(AIN2_PIN, 255);

  // Set motor to reverse direction
  digitalWrite(BIN1_PIN, LOW);
  // Set motor to no power
  analogWrite(BIN2_PIN, 0);


  // Explicitly configure I2C pins
  Wire.setPins(15, 16); // SDA = GPIO 15, SCL = GPIO 16
  Wire.begin();         // Initialize I2C

  // Set I2C clock speed to 50 kHz
  Wire.setClock(50000);  // Set I2C clock speed to 50 kHz

  Serial.println("I2C Scanner starting...");

  pinMode(12, OUTPUT);  
  digitalWrite(12, LOW);
  delay(10);  
  digitalWrite(12, HIGH);

  // NEEDED FOR SENSORS
  // Initialize the first VCNL4040 sensor on channel 0 
  selectPCA9548AChannel(0);
  if (!vcnl.begin()) {
    Serial.println("VCNL4040 on channel 0 not found!");
    while (1);  // Halt if the sensor is not found
  }
  Serial.println("VCNL4040 on channel 0 initialized.");
    // Set the VCNL4040 LED current (50mA)
  vcnl.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA);  // Set LED current to 50mA
  // Initialize the second VCNL4040 sensor on channel 1
  selectPCA9548AChannel(1);
  if (!vcnl.begin()) {
    Serial.println("VCNL4040 on channel 1 not found!");
    while (1);  // Halt if the sensor is not found
  }
  Serial.println("VCNL4040 on channel 1 initialized.");
    // Set the VCNL4040 LED current (50mA)
  vcnl.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA);  // Set LED current to 50mA
  // Initialize the second VCNL4040 sensor on channel 3
  selectPCA9548AChannel(3);
  if (!vcnl.begin()) {
    Serial.println("VCNL4040 on channel 3 not found!");
    while (1);  // Halt if the sensor is not found
  }
  Serial.println("VCNL4040 on channel 3 initialized.");
    // Set the VCNL4040 LED current (50mA)
  vcnl.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA);  // Set LED current to 50mA
    // Initialize the second VCNL4040 sensor on channel 7
  selectPCA9548AChannel(7);
  if (!vcnl.begin()) {
    Serial.println("VCNL4040 on channel 7 not found!");
    while (1);  // Halt if the sensor is not found
  }
  Serial.println("VCNL4040 on channel 7 initialized.");
    // Set the VCNL4040 LED current (50mA)
  vcnl.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA);  // Set LED current to 50mA
    // Initialize the second VCNL4040 sensor on channel 1
  selectPCA9548AChannel(2);
  if (!vcnl.begin()) {
    Serial.println("VCNL4040 on channel 2 not found!");
    while (1);  // Halt if the sensor is not found
  }
  Serial.println("VCNL4040 on channel 2 initialized.");
    // Set the VCNL4040 LED current (50mA)
  vcnl.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA);  // Set LED current to 50mA



  esc.attach(escPin, 1000, 2000); // pulse width limits

  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1000);   // Send low throttle to arm
  delay(3000);                   // Wait to initialize
  Serial.println("ESC Armed.");

}

// ---------- Global variables for sensor values ----------
uint16_t bottom_right_sensor = 0;
uint16_t bottom_left_sensor = 0;
uint16_t front_right_sensor = 0;
uint16_t front_middle_sensor = 0;
uint16_t front_left_sensor = 0;

// ---------- Time Tracking ----------
unsigned long lastBehaviorTime = 0;
const unsigned long behaviorInterval = 30000; // Run a behavior every 30 second

// ---------- Behavior States ----------
enum Behavior {NONE, SPOT_CLEAN, SPIRAL};
// , EDGE_FOLLOWER not used because can get stuck and fall easily
Behavior currentBehavior = SPOT_CLEAN;

// ---------- Helper Functions ---------
void updateSensors(){
  // Read proximity from the first sensor
  selectPCA9548AChannel(0);
  delay(10);  // Allow time for the channel to switch
  bottom_right_sensor = vcnl.getProximity();
  Serial.print("Proximity from bottom_right_sensor, front_right_sensor, bottom_left_sensor, front_middle_sensor: ");
  Serial.print(bottom_right_sensor);

  // Read proximity from the second sensor
  selectPCA9548AChannel(1);
  delay(10);  // Allow time for the channel to switch
  front_right_sensor = vcnl.getProximity();
  // Serial.print("Proximity from front_right_sensor: ");
  Serial.print(" , ");
  Serial.print(front_right_sensor);

  // Read proximity from the first sensor
  selectPCA9548AChannel(3);
  delay(10);  // Allow time for the channel to switch
  bottom_left_sensor = vcnl.getProximity(); 
  // Serial.print("Proximity from bottom_left_sensor: ");
  Serial.print(" , ");
  Serial.print(bottom_left_sensor);

  // Read proximity from the second sensor
  selectPCA9548AChannel(7);
  delay(10);  // Allow time for the channel to switch
  front_middle_sensor = vcnl.getProximity();
  // Serial.print("Proximity from front_middle_sensor: ");
  Serial.print(" , ");
  Serial.println(front_middle_sensor);

  // Read proximity from the second sensor
  selectPCA9548AChannel(2);
  delay(10);  // Allow time for the channel to switch
  front_left_sensor = vcnl.getProximity();
  Serial.print("Proximity from Sensor BROKEN: ");
  Serial.println(front_left_sensor);
}


bool detectCliff() {
  // Return true if any cliff sensor detects a drop
  if (bottom_left_sensor < 100 || bottom_right_sensor < 100){
    return true;
  } else {
    return false;
  }
}

int objectThreshold = 50;
bool detectObstacle() {
  // Return true if IR or bump sensor is triggered
  if (front_middle_sensor > objectThreshold || front_right_sensor > objectThreshold || front_left_sensor > objectThreshold){
    return true;
  } else {
    return false;
  }
}

void stopMotors() {
  // Set motor PWM to 0 or brake
  digitalWrite(AIN1_PIN, LOW);  // will stop (in forward direction)
  analogWrite(AIN2_PIN, 0);
  digitalWrite(BIN1_PIN, LOW);  // will stop (in forward direction)
  analogWrite(BIN2_PIN, 0);
}

void moveBackSlightly(){
  digitalWrite(AIN1_PIN, LOW);  // will spin reverse direction full speed
  analogWrite(AIN2_PIN, 255);
  digitalWrite(BIN1_PIN, LOW);  // will spin reverse direction full speed
  analogWrite(BIN2_PIN, 255);
  delay(500);
  stopMotors();
}

void turnLeft(){
  digitalWrite(AIN1_PIN, LOW);  // Reverse direction
  analogWrite(AIN2_PIN, 255);
  digitalWrite(BIN1_PIN, HIGH);  // Forward direction
  analogWrite(BIN2_PIN, 0);
}
void turnRight(){
  digitalWrite(AIN1_PIN, HIGH);  // Forward direction
  analogWrite(AIN2_PIN, 0);
  digitalWrite(BIN1_PIN, LOW);  // Revers direction
  analogWrite(BIN2_PIN, 255);
}

void goForward(){
  digitalWrite(AIN1_PIN, HIGH);  // Forward direction
  analogWrite(AIN2_PIN, 0);
  digitalWrite(BIN1_PIN, HIGH);  // Forward direction
  analogWrite(BIN2_PIN, 0);
}

void rotateRandom(){
  turnLeft();
  delay(random(200, 3001));
  stopMotors();
}
void rotateRandomRight(){
  digitalWrite(AIN1_PIN, HIGH);  // Forward direction
  analogWrite(AIN2_PIN, 0);
  digitalWrite(BIN1_PIN, LOW);  // Revers direction
  analogWrite(BIN2_PIN, 255);
  delay(random(200, 3001));
  stopMotors();
}

bool directionSet = false;
bool turnLeftDirection;  // 0 or 1
void avoidObstacle() {
  Serial.println("Avoiding obstacle...");
  stopMotors();
  delay(200);     // Small pause
  if (!directionSet){ //turn left if object in front of front_right_sensor
    if(front_right_sensor >= objectThreshold){
      turnLeftDirection = true;
    } else {
      turnLeftDirection = false;
    }
    directionSet = true;
  }


  if (turnLeftDirection && directionSet) {
    Serial.println("Turning left...");
    turnLeft();
    delay(200);
    stopMotors();
  } else if (!turnLeftDirection && directionSet) {
    Serial.println("Turning right...");
    turnRight();
    delay(200);
    stopMotors();
  }
}
// void avoidObstacle() {
//   Serial.println("Avoiding obstacle...");
//   stopMotors();
//   delay(200);     // Small pause

//   if (front_right_sensor >= objectThreshold){ //turn left if object in front of front_right_sensor
//     Serial.println("Turning left...");
//     turnLeft();
//     delay(200);
//     stopMotors();
//   } else {
//     Serial.println("Turning right...");
//     turnRight();
//     delay(200);
//     stopMotors();
//   }

//   // Keep turning until all front sensors are clear
//   Serial.println("Path clear, resuming...");
// }


bool edgeFollowInitialized = false;
Behavior lastBehavior = NONE;

enum EdgeFollowState {
  SEEK_EDGE,
  FOLLOW_LEFT,
  FOLLOW_RIGHT
};
EdgeFollowState edgeState = SEEK_EDGE;
const int cliffThreshold = 100;

void runBehavior() {

  // // Reset edge follow state only if we are switching to EDGE_FOLLOWER
  // if (currentBehavior != lastBehavior && currentBehavior == EDGE_FOLLOWER) {
  //   edgeState = SEEK_EDGE;
  //   edgeFollowInitialized = false;
  // }
  // lastBehavior = currentBehavior;

  switch (currentBehavior) {
    case SPOT_CLEAN:
      spotClean();
      break;
    case SPIRAL:
      Serial.println("Doing spiral...");
      spiralMotion();
      break;
    // case EDGE_FOLLOWER:
    //   Serial.println("Following edge...");
    //   followEdge();
    //   break;
    default:
      stopMotors();
  }
}

// State definitions
enum SpotCleanState {
  MOVING_FORWARD,
  TURNING_LEFT
};
SpotCleanState spotState = MOVING_FORWARD;
unsigned long spotLastChangeTime = 0;
// Set durations (in milliseconds)
const unsigned long forwardDuration = 1000;
const unsigned long turnDuration = 3700;
bool spotCleanModeInitialized = false;
bool spotCleanTurnDirection = true;
void spotClean() {
  unsigned long currentTime = millis();
  if (!spotCleanModeInitialized) {
    spotState = MOVING_FORWARD; // Every time spiralMotion() begins, set to 5000 interval for the first one
    spotLastChangeTime = currentTime;
    spotCleanTurnDirection = random(0, 2); // added to randomize turn direction
    spotCleanModeInitialized = true;
  }
  switch (spotState) {
    case MOVING_FORWARD:
      goForward();
      if (currentTime - spotLastChangeTime >= forwardDuration) {
        spotLastChangeTime = currentTime;
        spotState = TURNING_LEFT;
      }
      break;
    case TURNING_LEFT:
      if (spotCleanTurnDirection){ // added to randomize turn direction
        turnLeft();
        if (currentTime - spotLastChangeTime >= turnDuration) {
          spotLastChangeTime = currentTime;
          spotCleanTurnDirection = random(0, 2); // added to randomize turn direction
          spotState = MOVING_FORWARD;
        }
      } else {
        turnRight();
        if (currentTime - spotLastChangeTime >= turnDuration) {
          spotLastChangeTime = currentTime;
          spotCleanTurnDirection = random(0, 2); // added to randomize turn direction
          spotState = MOVING_FORWARD;
        }
      }
      break;
  }
}

// Spiral motion parameters
unsigned long lastSpiralSpeedUpdate = 0;
const unsigned long spiralSpeedInterval = 2000;  // Fixed: speed increases every 2 seconds
unsigned long lastRandomRotateTime = 0;
unsigned long randomRotateInterval = 0;          // Random: set each time a rotation is done
int spiralSpeedLeft = 180;                       // Base constant speed for left
int spiralSpeedRight = 0;                        // Start right speed low
const int spiralSpeedIncrement = 5;
const int spiralSpeedMax = 150;
bool spiralModeInitialized = false;


void spiralMotion() {
  unsigned long currentTime = millis();
  if (!spiralModeInitialized) {
    spiralSpeedLeft = 180;
    spiralSpeedRight = 0;
    randomRotateInterval = 5000; // Every time spiralMotion() begins, set to 5000 interval for the first one
    lastRandomRotateTime = currentTime;
    spiralModeInitialized = true;
  }
  // -------- 1. Increase Right Motor Speed Every 2 Seconds --------
  if (currentTime - lastSpiralSpeedUpdate >= spiralSpeedInterval) {
    lastSpiralSpeedUpdate = currentTime;
    spiralSpeedRight += spiralSpeedIncrement;
    if (spiralSpeedRight > spiralSpeedMax) {
      spiralSpeedRight = spiralSpeedMax;
    }
  }
  // -------- 2. Random Rotation Every Random Interval --------
  if (currentTime - lastRandomRotateTime >= randomRotateInterval) {
    rotateRandomRight();  // Or any random behavior you want
    lastRandomRotateTime = currentTime;
    randomRotateInterval = random(5000, 10000); // Pick next delay: 1–4 seconds
  }
  // -------- 3. Drive Motors --------
  digitalWrite(AIN1_PIN, HIGH);                  // Forward
  analogWrite(AIN2_PIN, spiralSpeedLeft);
  digitalWrite(BIN1_PIN, HIGH);                  // Forward
  analogWrite(BIN2_PIN, spiralSpeedRight);
}




// unsigned long lastEdgeRotateTime = 0;
// unsigned long edgeRandomInterval = 0;

// void followEdge() {


//   if (!edgeFollowInitialized) {
//     if (bottom_left_sensor < cliffThreshold && bottom_right_sensor >= cliffThreshold) {
//       edgeState = FOLLOW_LEFT;
//       edgeFollowInitialized = true;
//     } else if (bottom_right_sensor < cliffThreshold && bottom_left_sensor >= cliffThreshold) {
//       edgeState = FOLLOW_RIGHT;
//       edgeFollowInitialized = true;
//     } else {
//       edgeState = SEEK_EDGE;
//     }
//   }
//   switch (edgeState) {
//     case SEEK_EDGE:
//       if (bottom_left_sensor < cliffThreshold && bottom_right_sensor >= cliffThreshold) {
//         edgeState = FOLLOW_LEFT;
//         edgeFollowInitialized = true;
//       } else if (bottom_right_sensor < cliffThreshold && bottom_left_sensor >= cliffThreshold) {
//         edgeState = FOLLOW_RIGHT;
//         edgeFollowInitialized = true;
//       } else if (bottom_left_sensor < cliffThreshold && bottom_right_sensor < cliffThreshold) { //BOTH CLIFFS AT THE SAME TIME
//         moveBackSlightly(); // back off cliff
//         rotateRandom(); // rotate to reorient
//       } else {
//         goForward(); // no cliffs, keep going
//       }
//       break;

//     case FOLLOW_LEFT:
//       if (millis() - lastEdgeRotateTime >= edgeRandomInterval) {
//         moveBackSlightly();
//         rotateRandomRight();
//         lastEdgeRotateTime = millis();
//         edgeRandomInterval = random(5000, 10000);  // pick new random interval every time
//       }
//       if (bottom_left_sensor < cliffThreshold && bottom_right_sensor >= cliffThreshold) { //left sensor detecting cliff and right sensor on table
//         // curve right / rotate right wheel backwards while left motor stay stopped
//         digitalWrite(AIN1_PIN, HIGH);  // will stop (in forward direction)
//         analogWrite(AIN2_PIN, 255);
//         digitalWrite(BIN1_PIN, LOW);  // reverse right wheel
//         analogWrite(BIN2_PIN, 127);
//       } else if(bottom_left_sensor >= cliffThreshold && bottom_right_sensor >= cliffThreshold){ //both sensors on table
//         // curve left
//         digitalWrite(AIN1_PIN, HIGH);  // forward half speed
//         analogWrite(AIN2_PIN, 50);
//         digitalWrite(BIN1_PIN, HIGH);  // forward full speed
//         analogWrite(BIN2_PIN, 0);
//       } else if(bottom_left_sensor >= cliffThreshold && bottom_right_sensor < cliffThreshold){ //left sensor on table and right sensor detecting cliff
//         // curve right / rotate right wheel backwards while left motor stay stopped
//         digitalWrite(AIN1_PIN, HIGH);  // will stop (in forward direction)
//         analogWrite(AIN2_PIN, 255);
//         digitalWrite(BIN1_PIN, LOW);  // reverse right wheel
//         analogWrite(BIN2_PIN, 127);
//       } else if(bottom_left_sensor < cliffThreshold && bottom_right_sensor < cliffThreshold){ //both sensors off table
//         // curve right / rotate right wheel backwards while left motor stay stopped
//         digitalWrite(AIN1_PIN, HIGH);  // will stop (in forward direction)
//         analogWrite(AIN2_PIN, 255);
//         digitalWrite(BIN1_PIN, LOW);  // reverse right wheel
//         analogWrite(BIN2_PIN, 127);
//       }
//       break;

//     case FOLLOW_RIGHT:
//       if (millis() - lastEdgeRotateTime >= edgeRandomInterval) {
//         moveBackSlightly();
//         rotateRandom();
//         lastEdgeRotateTime = millis();
//         edgeRandomInterval = random(5000, 10000);  // pick new random interval every time
//       }
//       if (bottom_left_sensor < cliffThreshold && bottom_right_sensor >= cliffThreshold) { //left sensor detecting cliff and right sensor on table
//         // curve left / rotate left wheel backwards while right motor stay stopped
//         digitalWrite(AIN1_PIN, LOW);  // reverse left wheel
//         analogWrite(AIN2_PIN, 127);
//         digitalWrite(BIN1_PIN, HIGH);  // will stop (in forward direction)
//         analogWrite(BIN2_PIN, 255);
//       } else if(bottom_left_sensor >= cliffThreshold && bottom_right_sensor >= cliffThreshold){ //both sensors on table
//         // curve right
//         digitalWrite(AIN1_PIN, HIGH);  // forward full speed
//         analogWrite(AIN2_PIN, 0);
//         digitalWrite(BIN1_PIN, HIGH);  // forward half speed
//         analogWrite(BIN2_PIN, 50);
//       } else if(bottom_left_sensor >= cliffThreshold && bottom_right_sensor < cliffThreshold){ //left sensor on table and right sensor detecting cliff
//         // curve left / rotate left wheel backwards while right motor stay stopped
//         digitalWrite(AIN1_PIN, LOW);  // reverse left wheel
//         analogWrite(AIN2_PIN, 127);
//         digitalWrite(BIN1_PIN, HIGH);  // will stop (in forward direction)
//         analogWrite(BIN2_PIN, 255);
//       } else if(bottom_left_sensor < cliffThreshold && bottom_right_sensor < cliffThreshold){ //both sensors off table
//         // curve left / rotate left wheel backwards while right motor stay stopped
//         digitalWrite(AIN1_PIN, LOW);  // reverse left wheel
//         analogWrite(AIN2_PIN, 127);
//         digitalWrite(BIN1_PIN, HIGH);  // will stop (in forward direction)
//         analogWrite(BIN2_PIN, 255);
//       }
//       break;
//   }
// }

bool cliffTriggeredFlag = false;

void loop() {

  esc.writeMicroseconds(1150); // turn on vacuum

  updateSensors();


  // if ( front_left_sensor > 100 ){ //|| proximity4 > 100 || proximity5 > 100 || proximity6 > 100 BLINKING LIGHT
  //   digitalWrite(LED_PIN, HIGH); // Turn on LED if 0x60 is found
  //   delay(100);
  //   digitalWrite(LED_PIN, LOW); // Turn on LED if 0x60 is found
  //   delay(100);
  //   digitalWrite(LED_PIN, HIGH); // Turn on LED if 0x60 is found
  //   delay(100);
  //   digitalWrite(LED_PIN, LOW); // Turn on LED if 0x60 is found
  //   delay(100);
  //   digitalWrite(LED_PIN, HIGH); // Turn on LED if 0x60 is found
  //   delay(100);
  //   digitalWrite(LED_PIN, LOW); // Turn on LED if 0x60 is found
  //   delay(100);
  //   digitalWrite(LED_PIN, HIGH); // Turn on LED if 0x60 is found
  //   delay(100);
  // }
// // 1. Highest Priority: Cliff Detection
//   if (detectCliff() && currentBehavior != EDGE_FOLLOWER) {
//     stopMotors();
//     Serial.println("Cliff detected! Stopping.");
//     moveBackSlightly();
//     rotateRandom();
//     return; // Skip everything else
//   }
// 1. Highest Priority: Cliff Detection
  if (detectCliff() || cliffTriggeredFlag) {
    stopMotors();
    Serial.println("Cliff detected! Stopping.");
    cliffTriggeredFlag = true;
    if (bottom_left_sensor < cliffThreshold && bottom_right_sensor >= cliffThreshold) { //left sensor detecting cliff and right sensor on table
      // curve left / rotate left wheel backwards while right motor stay stopped
      digitalWrite(AIN1_PIN, LOW);  // reverse left wheel
      analogWrite(AIN2_PIN, 255);
      digitalWrite(BIN1_PIN, HIGH);  // will stop (in forward direction)
      analogWrite(BIN2_PIN, 255);
    } else if(bottom_left_sensor >= cliffThreshold && bottom_right_sensor >= cliffThreshold){ //both sensors on table
      // curve right
      digitalWrite(AIN1_PIN, HIGH);  // forward half speed
      analogWrite(AIN2_PIN, 127);
      digitalWrite(BIN1_PIN, HIGH);  // forward half speed
      analogWrite(BIN2_PIN, 127);
    } else if(bottom_left_sensor >= cliffThreshold && bottom_right_sensor < cliffThreshold){ //left sensor on table and right sensor detecting cliff
      // curve left / rotate left wheel backwards while right motor stay stopped
      digitalWrite(AIN1_PIN, HIGH);  // will stop (in forward direction)
      analogWrite(AIN2_PIN, 255);
      digitalWrite(BIN1_PIN, LOW);  // reverse right wheel
      analogWrite(BIN2_PIN, 255);
    } else if(bottom_left_sensor < cliffThreshold && bottom_right_sensor < cliffThreshold){ //both sensors off table
      moveBackSlightly();
      rotateRandom();
      cliffTriggeredFlag = false;
    }


    unsigned long currentMillis = millis();
    lastRandomRotateTime = currentMillis; // reset spiral randomRotateInterval
    spotCleanModeInitialized = false; // reset spotCleanMode (start from going forward)
    return; // Skip everything else
  }
  // 2. Medium Priority: Obstacle Avoidance
  if (detectObstacle()) {
    avoidObstacle();
    unsigned long currentMillis = millis();
    lastRandomRotateTime = currentMillis; // reset spiral randomRotateInterval
    spotCleanModeInitialized = false; // reset spotCleanMode (start from going forward)
    return; // Skip behavior
  } else{
    directionSet = false; // reset which direction to turn in avoidObstacle if no obstacles
  }

  // // 3. Low Priority: Behavior Logic
  // unsigned long currentMillis = millis();
  // if (currentMillis - lastBehaviorTime >= behaviorInterval) {
  //   lastBehaviorTime = currentMillis;
  //   currentBehavior = (Behavior)random(1, 3); // change behavior randomly
  // }
  // if (currentBehavior != SPIRAL) {
  //   spiralModeInitialized = false;
  // }
  // if (currentBehavior != SPOT_CLEAN) {
  //   spotCleanModeInitialized = false;
  // }
  
  runBehavior(); // Like spot clean, spiral, etc.


}

