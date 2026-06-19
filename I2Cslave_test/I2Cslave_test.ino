//ESP32-C3 I2C Slave Test Code
//test code can write and receive
#include <Wire.h>

#define I2C_SLAVE_ADDR 0x28
#define SDA_PIN 8
#define SCL_PIN 9

// Buffer for receiving data
volatile uint8_t receivedData[32];
volatile uint8_t dataLength = 0;
volatile bool dataReceived = false;

// Buffer for sending data
uint8_t sendData[32];
int count = 0;

// Called when the master sends data
void receiveEvent(int bytesin) {
  uint8_t len = 0;

  while (Wire.available()) {
    if (len < sizeof(receivedData)) {
      receivedData[len++] = Wire.read();
    } else {
      Wire.read();  // Discard excess data
    }
  }
  dataLength = len;
  dataReceived = true;
}

// Called when the master requests data
void sendCount() {
  Wire.write(count++);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin((uint8_t)I2C_SLAVE_ADDR, SDA_PIN, SCL_PIN, 40000);

  // Register event handlers
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendCount);

  Serial.println("ESP32 I2C Slave initialized");
  Serial.printf("Address: 0x%02X, SDA: %d, SCL: %d\n", I2C_SLAVE_ADDR, SDA_PIN, SCL_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (dataReceived) {
      Serial.printf("Received data: %d\n", receivedData[0]);
      dataReceived = false;            // Clear flag
    }

}
