
#pragma once
#include "PinNames.h"
#include "Timer.h"
#include "mbed.h"
#include "rplidar.h"
//#include <cstdio>

//Serial PC(SERIAL_TX,SERIAL_RX); //Rx Tx pins of the USB (used to write in the serial monitor);

class Print
{
public:
    Serial& _PC;
    Print(Serial& PC) : _PC(PC) {} ;
    ~Print();

void printData(int angle, int sizeOfPrint, float* data);
void printData(int startAngle, int angle, int sizeOfPrint, float* data);
void getPrintData(int angle, int sizeOfPrint, RPLidar lidar, int readTime);


};