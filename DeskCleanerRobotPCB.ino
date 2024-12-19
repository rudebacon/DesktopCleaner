




#include <Wire.h>
#include <Adafruit_VCNL4040.h>  // Include the VCNL4040 library

byte busStatus;
bool i2cDeviceFound = false; // Flag to indicate if an I2C device is detected

#define LED_PIN 8 // GPIO 8 for LED

#define MODE_PIN 40  // GPIO 40 connected to MODE
#define AIN1_PIN 41  // GPIO 41 connected to AIN1 (PHASE)
#define AIN2_PIN 42  // GPIO 42 connected to AIN2 (ENABLE)

#define PCA9548A_ADDR 0x70  // I2C address of the PCA9548A multiplexer

Adafruit_VCNL4040 vcnl;  // Create an instance of the VCNL4040 sensor

void setup() {
  Serial.begin(115200);

  // Configure LED pin as output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially

  // // Configure motor pins as outputs //UNCOMMENT MOTOR STUFF LATER
  // pinMode(MODE_PIN, OUTPUT);
  // pinMode(AIN1_PIN, OUTPUT);
  // pinMode(AIN2_PIN, OUTPUT);
  // // Set MODE to PHASE/ENABLE mode
  // digitalWrite(MODE_PIN, LOW);
  // // Set motor to forward direction
  // digitalWrite(AIN1_PIN, HIGH);
  // // Set motor to full power
  // digitalWrite(AIN2_PIN, LOW);

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

  // Initialize the first VCNL4040 sensor on channel 0
  selectPCA9548AChannel(0);
  if (!vcnl.begin()) {
    Serial.println("VCNL4040 on channel 0 not found!");
    while (1);  // Halt if the sensor is not found
  }
  Serial.println("VCNL4040 on channel 0 initialized.");

  // Initialize the second VCNL4040 sensor on channel 1
  selectPCA9548AChannel(1);
  if (!vcnl.begin()) {
    Serial.println("VCNL4040 on channel 1 not found!");
    while (1);  // Halt if the sensor is not found
  }
  Serial.println("VCNL4040 on channel 1 initialized.");

  // Set the VCNL4040 LED current (50mA)
  vcnl.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA);  // Set LED current to 50mA
}

void loop() {

    // Select the channel for VCNL4040 using the PCA9548A expander
  selectPCA9548AChannel(0);  // Ensure the correct channel is selected


  i2cDeviceFound = false; // Reset the flag for each scan


// Scan I2C bus for devices
bool device60Found = false; // Flag for detecting 0x60
for (int i2cAddress = 0x08; i2cAddress < 0x78; i2cAddress++) {
  Wire.beginTransmission(i2cAddress);
  busStatus = Wire.endTransmission();
  if (busStatus == 0x00) {
    Serial.print("I2C Device found at address: 0x");
    Serial.println(i2cAddress, HEX);
    if (i2cAddress == 0x60) {
      device60Found = true; // Set flag if 0x60 is found
    }
  }
}

if (device60Found) {
  digitalWrite(LED_PIN, HIGH); // Turn on LED if 0x60 is found
  Serial.println("I2C device 0x60 detected. LED ON.");
} else {
  digitalWrite(LED_PIN, LOW); // Turn off LED if 0x60 is not found
  Serial.println("I2C device 0x60 not detected. LED OFF.");
}

Serial.println("Scan complete.");






  // // Select the channel for VCNL4040 using the PCA9548A expander
  // selectPCA9548AChannel(0);  // Ensure the correct channel is selected


// Wire.beginTransmission(PCA9548A_ADDR); //THIS MAKES IT NOT WORK!!!!!! IT'S SUPPOSED TO ONLY CHECK BUT MAKES IT NOT WORK FOR SOME REASON
// Wire.write(0x00);  // Request channel register
// Wire.endTransmission();
// Wire.requestFrom(PCA9548A_ADDR, 1);
// uint8_t activeChannel = Wire.read();
// Serial.print("Active PCA9548A Channel: ");
// Serial.println(activeChannel, BIN);

  // // Read proximity from VCNL4040
  // uint16_t proximity = vcnl.getProximity();  // Get proximity value from VCNL4040
  // // Debugging: Print the raw proximity value
  // // Serial.print("Raw VCNL4040 Proximity: ");
  // // Serial.println(proximity);
  // // Check if the proximity value is valid (not 0)
  // if (proximity == 0) {
  //   Serial.println("Proximity value is 0. Ensure the sensor is properly calibrated and there's an object in range.");
  // }
  // // Print the proximity value to the serial monitor
  // Serial.print("VCNL4040 Proximity0:");
  // Serial.println(proximity);


  // Read proximity from the first sensor
  selectPCA9548AChannel(0);
  delay(10);  // Allow time for the channel to switch
  uint16_t proximity1 = vcnl.getProximity();
  Serial.print("Proximity from Sensor 1: ");
  Serial.println(proximity1);

  // Read proximity from the second sensor
  selectPCA9548AChannel(1);
  delay(10);  // Allow time for the channel to switch
  uint16_t proximity2 = vcnl.getProximity();
  Serial.print("Proximity from Sensor 2: ");
  Serial.println(proximity2);



  // // Read the logic level of GPIO 12 (i2c expander reset (should be HIGH)) //DEBUGGING I2C EXPANDER IF RESET IS PROBLEM
  // int logicLevel = digitalRead(12);  
  // // Print the current logic level to the serial monitor
  // if (logicLevel == HIGH) {
  //   Serial.println("GPIO 12 is HIGH");
  // } else {
  //   Serial.println("GPIO 12 is LOW");
  // }

  delay(500); // Wait before the next scan (faster updates)
}

// Function to select the channel on the PCA9548A expander
void selectPCA9548AChannel(uint8_t channel) {
  if (channel > 7) return; // Ensure valid channel (0-7)
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);  // Activate the desired channel
  Wire.endTransmission();
}
