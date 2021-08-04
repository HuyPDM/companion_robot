#ifndef ROBOT_MOVE_H
#define ROBOT_MOVE_H
#pragma once
#include <iostream>
// for delay function.
#include <chrono> 
#include <thread>

// for signal handling
#include <signal.h>
#include <math.h>
#include <../include/JetsonGPIO.h>

#define STOP true
#define RUN false

#define FORWARD true
#define BACKWARD false

#define MaxRight 100
//#define MaxLeft 95

using namespace std;
using namespace GPIO;

//void delay(int s);
void signalHandler (int s);
int Distance(int knowWidth, int knowHeight, int frame_width, int frame_height);
/*GPIO::PWM CreatePWMObject(int output_pin);
void SetupMotor();

void MotorLeft(bool isForward, int Speed);
void MotorRight(bool isForward, int Speed);

void Move(int frameWidth,int frameHeight, int recX, int recWidth, int recHeight);
void Cleanup();*/
#endif
