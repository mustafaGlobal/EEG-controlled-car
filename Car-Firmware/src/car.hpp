#ifndef CAR_HPP
#define CAR_HPP

#include <Arduino.h>

// Left Motor 
#define ENA 6
#define IN1 8
#define IN2 7


// Right Motor
#define ENB 5
#define IN3 4
#define IN4 3

class Car
{
  private:
    void motorRight(int speed);
    void motorLeft(int speed);

  public:
    Car() = default;

    void begin();
    void stop();

    void forward(int throttle = 255);
    void backward(int throttle = 255);
    void left(int throttle = 255);
    void right(int throttle = 255);

    void run(int left, int right);

};



#endif