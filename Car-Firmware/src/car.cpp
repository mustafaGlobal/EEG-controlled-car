#include "car.hpp"

void Car::begin()
{
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void Car::motorLeft(int speed)
{
  speed = speed > 255 ? 255 : speed;
  speed = speed < -255 ? -255 : speed;
  if(speed>0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  }
  else if (speed == 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  }
}

void Car::motorRight(int speed)
{
  speed = speed > 255 ? 255 : speed;
  speed = speed < -255 ? -255 : speed;
  if(speed>0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  }
  else if (speed == 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  }
}

void Car::run(int left, int right)
{
  motorLeft(left);
  motorRight(right);
}

void Car::forward(int throttle)
{
  motorLeft(throttle);
  motorRight(throttle);
}

void Car::backward(int throttle)
{
  motorLeft(-throttle);
  motorRight(-throttle);
}

void Car::left(int throttle)
{
  motorLeft(-throttle/10);
  motorRight(throttle);
}

void Car::right(int throttle)
{
  motorLeft(throttle/10);
  motorRight(-throttle);
}

void Car::stop()
{
  motorLeft(0);
  motorRight(0);
}
