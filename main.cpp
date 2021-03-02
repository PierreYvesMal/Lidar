/*
 *  RPLIDAR SDK for Mbed
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "RawSerial.h"
#include "mbed.h"
#include "rplidar.h"
#include "rplidar_driver.h"
#include "print.h"
#include <cstdio>

/*
 *
 * NOTICE : The led2 of the nucleo is coordinated with motor lidar's state
 * 1 : motor on
 * 0 : motor off
 *
 */

// <VARIABLES> 
RPLidar lidar;
const int BAUDERATE = 115200;
int setLidar = 1; //Used to first initialize the lidar
int angle = 90; //Divide 360° in parts to lessen the errors

PwmOut VMO(PB_3); //lidar's PWM pin
//RawSerial se_lidar(PA_11, PA_12, BAUDERATE); //F401RE Tx Rx pins where is the lidar connected and the bauderate used for the serial communication
RawSerial se_lidar(PC_4,PC_5, BAUDERATE); //L476RG

Timer tim; //Timer

DigitalIn BP(USER_BUTTON);
DigitalOut Led2(LED2);

Serial PC(SERIAL_TX,SERIAL_RX); //Rx Tx pins of the USB (used to write in the serial monitor)

//Thread thread;


// ???????????
int buf = 0;

// </VARIABLES>

int main(){  
    Print printer(PC);

// <GLOBAL VARIABLES INIT>
    lidar.begin(se_lidar); //Initialize the serial port used for the lidar
    lidar.setAngle(0,angle);

    PC.baud(BAUDERATE); //initialize the bauderate in the serial monitor
    PC.printf("READY \n");
    //PC.attach(&Rx_interrupt, Serial::RxIrq);  //?????????????

    VMO = 0; //No pulse, motor off
    Led2.write(VMO);
// </GLOBAL VARIABLES INIT>

// <LOCAL VARIABLES & INIT>
    int state, prevState, ledState;
    prevState = !VMO; //motor was previously on
    state = ledState = VMO; //motor is now off

    int readTime = 2'000;   //Reading data ()

    //For print
    int sizeOfPrint = 20;   
    int step = angle/sizeOfPrint;


    if(setLidar == 1){
        lidar.setLidar();
        setLidar = 0;
    }
// </LOCAL VARIABLES & INIT>

// <DEBUG>

    //thread.start(callback(&lidar, &RPLidar::waitPoint));
    //thread.start(&lidar, &RPLidar::waitPoint);

    //thread.start(callback(&RPLidar::waitPoint, &lidar));

    //wait_us(2'000'000);


    ///PREHEAT
    VMO = 0;
    //wait(60);   //Supposed to preheat 2 min
   
    //wait(2);
    state=0;
    while(true) {  //Infinite loop
        
        state = BP.read(); //We read the button
        //!\\ Had to change after a reboot from state==1 (which is logic) to state ==0 for some reason
        if(state != prevState && state == 0){ //If the two states are different and the button is pressed
            //tim.start();

            //ledState = !ledState; //We invert the state of the led for debug
            //Led2.write(ledState); //We send the value to the pin of the led

            /*
            //Sampling
            buf = tim.read_ms();
            while(tim.read_ms()-buf<=1'000){
                //PC.printf("%d\n",tim.read_ms()-buf);
                lidar.waitPoint(0);
            }
            */

            VMO=0.9f;   //Sets motor speed
            PC.printf("VMO = %.1f\t Angle = %d\t readTime = %d\n",(float)VMO,angle,readTime);
            printer.getPrintData(angle, sizeOfPrint, lidar, readTime);//This one does everything in one go (gets data then prints it)

        }   
    }
}