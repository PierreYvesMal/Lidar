
#include "print.h"

void Print::printData(int angle, int sizeOfPrint, float* data)
{
    
    int startAngle =0;
    int step = angle/sizeOfPrint;

    //Printing
    _PC.printf("Distance\t");
    for(int j = startAngle; j < startAngle + angle; j+=step) _PC.printf("%.0f\t", data[j]); //in cm 
    _PC.printf("\nAngle\t\t");
    for(int j = startAngle; j < startAngle + angle; j+=step) _PC.printf("%.0f\t",data[j]);
    _PC.printf("\n");
}

void Print::printData(int startAngle, int angle, int sizeOfPrint, float* data)
{
    
    int step = angle/sizeOfPrint;

    //Printing
    _PC.printf("Distance\t");
    for(int j = startAngle; j < startAngle + angle; j+=step) _PC.printf("%.0f\t", data[j]); //in cm 
    _PC.printf("\nAngle\t\t");
    for(int j = startAngle; j < startAngle + angle; j+=step) _PC.printf("%.0f\t",data[j]);
    _PC.printf("\n");
}

void Print::getPrintData(int angle, int sizeOfPrint, RPLidar lidar, int readTime)
{
    int startAngle=0;
    int buf=0;
    int step = angle/sizeOfPrint;
    Timer tim;
    //tim.start();
    _PC.printf("Timer started\n");

    while(startAngle<360)
    {                
        lidar.setAngle(startAngle, startAngle+angle);
        lidar.setLidar();

        //Getting data
        buf = tim.read_ms();
        while(tim.read_ms()-buf<=readTime){//Crashes after around 400
            _PC.printf("tim.read_ms() : %d\n",tim.read_ms());
            lidar.waitPoint(0);//Causes the crash
        }

        //Printing
        _PC.printf("Distance\t");
        for(int j = startAngle; j < startAngle + angle; j+=step) _PC.printf("%.0f\t", lidar.Data[j].distance); //in cm 
        _PC.printf("\nAngle\t\t");
        for(int j = startAngle; j < startAngle + angle; j+=step) _PC.printf("%.0f\t",lidar.Data[j].angle);
        _PC.printf("\n");
            
        startAngle+=angle;
        _PC.printf("\n\n");

    }
} 

