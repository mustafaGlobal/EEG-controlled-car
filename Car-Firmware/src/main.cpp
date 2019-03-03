#include <Arduino.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#include "car.hpp"

Car car;
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

message received_data;

void radio_init()
{
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
}

void setup()
{
  Serial.begin(115200);
  car.begin();
  radio_init();
  radio.openReadingPipe(1, 0xF0F0F0F0F0);
  radio.startListening();
  car.stop();
}

unsigned long lastRecvTime = 0;

void receive_the_data()
{
  while (radio.available())
  {
    radio.read(&received_data, sizeof(message));
    lastRecvTime = millis();
  }
}

void reset_the_Data()
{
  received_data.yaw = 0;
  received_data.roll = 0;
  received_data.pitch = 0;
  received_data.signal_strength = 200;
  received_data.attention = 0;
  received_data.meditation = 0;
}

void loop()
{
  receive_the_data();
  unsigned long now = millis();
  if (now - lastRecvTime > 20000)
  {
    reset_the_Data();
  }

  byte attention_threshold = 40;

  if(received_data.signal_strength < 20)
  {

    if (received_data.pitch > -60 && received_data.pitch < -15 &&
      received_data.attention >= attention_threshold) // Glava naprijed 
      { 
        car.forward();
        delay(100);
        car.stop();
      }

       if (received_data.pitch > 20 && received_data.pitch < 60 &&
      received_data.attention >= attention_threshold) // Glava nazad
      { 
        car.backward();
        delay(100);
        car.stop();
      }

      if (received_data.roll > -140 && received_data.roll < -90 && 
      received_data.attention >= attention_threshold )
      {
        car.left();
        delay(100);
        car.stop();
      }
      if (received_data.roll > -60 && received_data.roll < -30 && 
      received_data.attention >= attention_threshold )
      {
        car.right();
        delay(100);
        car.stop();
      }
  }
  else
  {
    car.stop();
  }
  
}