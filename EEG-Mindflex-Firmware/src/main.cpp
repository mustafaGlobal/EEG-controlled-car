#include <Arduino.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Brain.h"

Brain brain(Serial);

#define INTERRUPT_PIN 2

RF24 radio(9, 10);

struct message
{
  int yaw;
  int pitch;
  int roll;
  unsigned char signal_strength;
  unsigned char attention;
  unsigned char meditation;
};

message sent_data;

void radio_init()
{
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  sent_data.yaw = 0;
  sent_data.pitch = 0;
  sent_data.roll = 0;
  sent_data.signal_strength = 200;
  sent_data.attention = 0;
  sent_data.meditation = 0;
}

MPU6050 mpu;

bool firstboot;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
  }
  radio_init();
  radio.openWritingPipe(0xF0F0F0F0F0);
  firstboot = true;
}

void loop()
{

  if (!dmpReady)
    return;

  while (!mpuInterrupt && fifoCount < packetSize)
  {
    if (mpuInterrupt && fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
    }
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
  {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
  }
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    sent_data.yaw = ypr[0] * 180 / M_PI;
    sent_data.pitch = ypr[1] * 180 / M_PI;
    sent_data.roll = ypr[2] * 180 / M_PI;

    if(brain.update())
    {
      sent_data.signal_strength = brain.readSignalQuality();
      sent_data.attention = brain.readAttention();
      sent_data.meditation = brain.readMeditation();
    }
    
    radio.write(&sent_data, sizeof(message));  
  }
}
