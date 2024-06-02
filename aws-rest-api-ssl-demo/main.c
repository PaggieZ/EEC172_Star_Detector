// Team members: Erin Le, Peggy Zhu

//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution. 
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
//
//*****************************************************************************


//*****************************************************************************
//
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "rom.h"
#include "rom_map.h"

#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "hw_nvic.h"
#include "i2c_if.h"
#include "i2c_if.c"



#include "hw_gpio.h"
#include "interrupt.h"

#include "gpio.h"
#include "systick.h"

#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "pin.h"
#include "spi.h"





//Common interface includes
#include "pin_mux_config.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "math.h"


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

//Adafruit
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "oled_test.h"

// Custom includes
#include "utils/network_utils.h"


/////////////////

// Standard includes

#include <string.h>
#include <stdlib.h>




#define GRAY 0x8410

#define APPLICATION_VERSION     "0.0.0"
#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100
#define APP_NAME                "Star Chart"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}



#define CONTROL_MODE 0  // control mode 0 for accelerometer control, 1 for remote control

//communication with OLED
#define MOSI_Pin Pin_07
#define CLK_Pin Pin_05
#define DC_Pin 0x80 // Pin 45
#define Reset_Pin 0x10 // Pin 18
#define OC_Pin 0x2 // Pin 8
#define SPI_IF_BIT_RATE 100000

//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                19    /* Current Date */
#define MONTH               5     /* Month 1-12 */
#define YEAR                2024  /* Current year */
#define HOUR                9    /* Time - hours */
#define MINUTE              10    /* Time - minutes */
#define SECOND              0     /* Time - seconds */


#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "av2lay1g526bx-ats.iot.us-east-1.amazonaws.com" // CHANGE ME
#define GOOGLE_DST_PORT       8443


#define POSTHEADER "POST /things/Peggy_CC3200Board/shadow HTTP/1.1\r\n"             // CHANGE ME
#define GETHEADER "GET /things/Peggy_CC3200Board/shadow HTTP/1.1\r\n"
#define HOSTHEADER "Host: av2lay1g526bx-ats.iot.us-east-1.amazonaws.com\r\n"  // CHANGE ME
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"var\" :\""                                           \
                        "Hello phone, "                                     \
                        "message from PC via AWS IoT!"                  \
                        "\"\r\n"                                            \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int set_time();
static void BoardInit(void);
static int http_post(int);


#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define MAX_STRING_LENGTH    80


#define SPI_IF_BIT_RATE 100000


// some helpful macros for systick

#define SYSCLKFREQ 80000000ULL
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 60ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 4800000UL

volatile int systick_elapsed = 0;
volatile int reset_count = 0;
volatile int buffer[100] = {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2};
volatile int buffer_ind = 0;
char send_buffer[MAX_STRING_LENGTH];   //buffer for messages to be sent out
int send_index = 0;
int press_count = 0;
volatile int prev_falling_edge_time = 0;



// function declarations

void format_and_send(int iTLSSockID);

static int http_get(int iTLSSockID);
static int http_post(int iTLSSockID);

static int http_post_msg(int iTLSSockID, char* data);

static inline void SysTickReset(void) {
    HWREG(NVIC_ST_CURRENT) = 1;
    //systick_expired = 0;
    systick_elapsed = 0;

    //reset = 1;
}

//pin 63 info
#define IR_GPIO_PORT GPIOA1_BASE
#define IR_GPIO_PIN 0x1
volatile int systick_delta = 0; //formerly ulsystick_delta_us

// GPIO handler

static void GPIOA0IntHandler(void) {
    //static bool prev_state = 1;
    //Report("buffer_ind: %d\n", buffer_ind);


    // disable systick at the beginning and enable it at the end
    SysTickIntDisable();
    //clear interrupt
    // get and clear status
    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus(IR_GPIO_PORT, true);
    MAP_GPIOIntClear(IR_GPIO_PORT, ulStatus);


    unsigned long systick_val = MAP_SysTickValueGet();


    systick_delta = 4800000 - systick_val;


    if (systick_delta < 96000){ //less than 1.13 ms == 0. round up from 1.125 to allow for some buffer
        buffer[buffer_ind] = 0;
        buffer_ind++;

    }
    else if (systick_delta >= 96000 && systick_delta < 240000){ // between 0 threshold=1.125 ms and 1 threshold=2.25 ms
        buffer[buffer_ind] = 1;
        buffer_ind++;


    }
    else{   // long pulse; reset value. clear buffer (though may not be necessary)

        int i;
        for (i = 0; i < buffer_ind; i++){
            buffer[i]= 3;
        }
        buffer_ind=0;
        SysTickReset();
    }
    SysTickIntEnable();
    SysTickReset();
    return;
}

static void SysTickHandler(void) {
    systick_elapsed = 1; // timer has rolled over
    reset_count++;
}


volatile int g_iCounter = 0;
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif




static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t        CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}



// return the data value
// 0-9 is the corresponding numerical value
// 11 is enter
// 12 is delete
// -1 is bad data parity
// -2 is bad addr parity
// byteOne is the 8 data bits, byteTwo is the 8 inverse data bits
int checkData(int dataByteOne[8], int dataByteTwo[8], int addrByteOne[8], int addrByteTwo[8]) {
    int dataVal = 0;
    int i = 0;
    // check parity
    for (i = 0; i < 8; i++) {
        if ((dataByteOne[i] + dataByteTwo[i]) != 1) {
            return -1;
        }
    }
    for (i = 0; i < 8; i++) {
        if ((addrByteOne[i] + addrByteTwo[i]) != 1) {
            return -2;
        }
    }

    bool isEnter = true;
    bool isDelete = true;
    int enterByte[8] = {0, 0, 0, 0, 0, 0, 0, 1};

    int deleteByte[8] = {1, 1, 1, 0, 1, 0, 0, 0};
    for (i = 0; i < 8; i++) {
        if (dataByteOne[i] != enterByte[i]) {
            isEnter = false;
        }
        if (dataByteOne[i] != deleteByte[i]) {
            isDelete = false;
        }
    }
    if (isEnter) {
        return 11;
    } else if (isDelete) {
        return 12;
    } else {
        for (i = 0; i < 4; i++) {
            dataVal = dataVal + dataByteOne[i] * pow(2, i);
        }
        if ((dataVal >= 0) | (dataVal <= 0)) {
            return dataVal;
        }
        return -1;
    }


}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}




//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}


//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

struct Star{
    char* name;
//    float ra[3];
//    float dec[3];
    float x;
    float y;
    int magnitude;
};

unsigned char accel_reg_buffer[256] = {0};
// Based on ProcessReadRegCommand(), except it takes three parameters instead of parsing a string
unsigned char newReadReg(unsigned char ucDevAddr, unsigned char ucRegOffset, unsigned char ucRdLen){
//        Message("In newreadreg");
        // Write the register address to be read from.
        // Stop bit implicitly assumed to be 0.
        RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));
        //Message("Past if write\n");

        // read acceleration data into accel_reg_buffer, which is a global array. Only need to read one byte
        RET_IF_ERR(I2C_IF_Read(ucDevAddr, &accel_reg_buffer[0], ucRdLen));
        //Message("Past if read\n");
//        Report("%c", accel_reg_buffer[0]);
        return 0;

}




void main() {
    long lRetVal = -1;
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();


    // Enable the SPI module clock

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //Reset peripheral (don't know if needed)

    MAP_PRCMPeripheralReset(PRCM_GSPI);
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                         SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                         (SPI_SW_CTRL_CS |
                         SPI_4PIN_MODE |
                         SPI_TURBO_OFF |
                         SPI_CS_ACTIVEHIGH |
                         SPI_WL_8));

    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);

    //enable GPIO and SysTick interrupts
    // configure SysTick
        //systick_wrapped = 1;
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);
    MAP_SysTickIntRegister(SysTickHandler);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    // Register GPIO interrupt handler
    MAP_GPIOIntRegister(IR_GPIO_PORT, GPIOA0IntHandler);

    // Enter the GPIO interrupt handler only on a falling edge
    MAP_GPIOIntTypeSet(IR_GPIO_PORT, IR_GPIO_PIN, GPIO_FALLING_EDGE);

    // Clear GPIO interrupt
    uint64_t ulStatus = MAP_GPIOIntStatus(IR_GPIO_PORT, false);
    MAP_GPIOIntClear(IR_GPIO_PORT, ulStatus);
    // Enable interrupts when receiving input from IR receiver.
    MAP_GPIOIntEnable(IR_GPIO_PORT, IR_GPIO_PIN);


    InitTerm();
    ClearTerm();
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    UART_PRINT("My terminal works!\n\r");


    Message("Initializing OLED...\n\r");
    Adafruit_Init();

    Message("Done initializing OLED\n\r");

    fillScreen(BLACK);

    // code for connecting to aws
    // initialize global default app configuration
//    g_app_config.host = SERVER_NAME;
//    g_app_config.port = GOOGLE_DST_PORT;

    //Connect the CC3200 to the local access point
//    lRetVal = connectToAccessPoint();
//    //Set time so that encryption can be used
//    lRetVal = set_time();
//    if(lRetVal < 0) {
//        UART_PRINT("Unable to set time in the device");
//        LOOP_FOREVER();
//    }
    //Connect to the website with TLS encryption
//    lRetVal = tls_connect();
//    if(lRetVal < 0) {
//        ERR_PRINT(lRetVal);
//    }
    // send out POST and GET request before accepting user input
    //http_post(lRetVal);
    //http_get(lRetVal);

    Report("period length: %d\n\r",SYSTICK_RELOAD_VAL );
    //int prev_val = -1;

    // code that opens a csv file and reads in data.
    // data format is: RA hour, RA minute, RA second, DEC degree, DEC arcminute, DEC arcsecond
    // RA is right ascension and is equivalent to longitude for celestial coordinates
    // DEC is declination and is equivalent to latitude for celestial coordinates
    // an arcminute is 1/60 of a degree, a arcsecond is 1/60 of an arcminute

    //Referenced https://www.w3schools.com/c/c_files_read.php

    FILE* star_data;
    star_data = fopen("/Users/erinle/Desktop/spring2024/eec172/lab5/aws-rest-api-ssl-demo/sample-data-name-x-y-mag.csv", "r");

    struct Star star_array[200];
    int star_array_ind = 0;
    char curr_line[150];
    char delim[2] = ",";
    char* val;
    //Referenced https://www.geeksforgeeks.org/strtok-strtok_r-functions-c-examples/
    fgets(curr_line, 150, star_data); //get rid of first header line
    if (star_data != NULL){
        while(fgets(curr_line, 150, star_data)){
            //Report("Current line: %s", curr_line);
            // for now, use x and y data calculated from google sheet


            val = strtok(curr_line, delim);
            star_array[star_array_ind].name = val;
            val = strtok(NULL, delim);
            star_array[star_array_ind].x = atof(val);
            val = strtok(NULL, delim);
            star_array[star_array_ind].y = atof(val);

            val = strtok(NULL, delim);
            star_array[star_array_ind].magnitude = atoi(val);

//            Report("name: %s, x: %f, y: %f, magnitude: %d", star_array[star_array_ind].name, star_array[star_array_ind].x,
//                   star_array[star_array_ind].y, star_array[star_array_ind].magnitude);
            // read in right ascension and declination data
            //star_array[star_array_ind].ra[0] = atof(val);
//            int j = 0;
//            while (j = 1; j < 3; j++){
//                star_array[star_array_ind].ra[j] = atof(val);
//                val = strtok(NULL, delim); //keep performing strtok from after the current token
//            }
//            j = 0;
//            while(val != 0){
//                star_array[star_array_ind].dec[j] = atof(val);
//                val = strtok(NULL, delim);
//                j++;
//            }
//
////            while(val != 0){
////                Report("Val: %s\n", val);
////                val = strtok(NULL, delim); //keep performing strtok from after the current token
////            }

            // convert to x and y
            // first, do stereographic projection from ra and dec to flat polar coords
//            float projection_theta;
//            float projection_rad;
            star_array_ind++;
        }
        Message("Done reading in data");

    }
    else{
        Message("Couldn't open file");

    }



    // coordinates are in terms of the "star map", of which the OLED will only show a small portion
    // at a time
    int oled_x = -WIDTH/2;   //coordinates of the top left corner of the OLED. Start it
    int oled_y = -HEIGHT/2; // so that the center of the OLED is at the center of the "star map"
    int one_g = 0x42;
    int x_accel = 0;
    int x_dir = 1;
    int y_dir = 1;
    int y_accel = 0;
    int move_const = 10;
    float max_rad = 640;
//    int prev_oled_x = -WIDTH/2;
//    int prev_oled_y = -HEIGHT/2;    //start the
//    int prev_ball_color = 0xCCCCCC;
    int prev_star_screen_x[100];
    int prev_star_screen_y[100];
    int prev_screen_ind = 0;
    int prev_prev_star_screen_x[100];
    int prev_prev_star_screen_y[100];
    int prev_prev_screen_ind = 0;




    // code from lab 3 that decodes the IR signal
    buffer_ind = 0;
    while(1){

        Report("x: %d, y: %d\n", x_accel, y_accel);





        //fillScreen(BLACK);
        //Message("before read reg\n");
        //read and set current value of x acceleration
        newReadReg((unsigned char)0x18, (unsigned char)0x3, (unsigned char)1);
        x_accel = accel_reg_buffer[0];

        //read and set current value of y acceleration
        newReadReg((unsigned char)0x18, (unsigned char)0x5, (unsigned char)1);
        y_accel = accel_reg_buffer[0];
        //Report("x_accel: %d, y_accel: %d\n", x_accel, y_accel);
        //Message("Past read reg\n");
        //Datasheet p 16: Accel data is represented as 8 bit, 2s complement
        //z accel when resting on table is ~1g, readreg command gives 0x40 to 0x41. Round up to 0x42

//                if (x_accel <= 5 && y_accel <= 5){
//                    int k;
//                    for (k = 0; k < prev_screen_ind; k++){
//                        drawCircle(prev_star_screen_x[k], prev_star_screen_y[k], 1, WHITE);
//                    }
//                    continue;
//                }

        //Extract the direction of tilt and the magnitude of the acceleration values.
        // x_dir and y_dir are -1 if the MCU is tilted to the left/ downward, if the micro USB port is at the top left
        int mask = 0x80;
        if (mask & x_accel){ //Check for a negative value
            x_dir = -1;
            //convert from 2s complement
            x_accel = ~x_accel; //bitwise not
            x_accel += 1;
            x_accel = x_accel & 0xff;

        } else{
            x_dir = 1;
        }

        if (mask & y_accel){
            y_dir = -1;
            //convert from 2s complement
            y_accel = ~y_accel; //bitwise not
            y_accel += 1;
            y_accel = y_accel & 0xff;    //discard carry bit (if any) from MSB
        }
        else{
            y_dir = 1;
        }


        //TODO change cursor speed based on accelerometer measurements

        // set the new x position. Start with the current position, then decide how much to change the position based on accelerometer
        // measurement (magnitude and direction). Move_const changes the speed at which the ball moves.

        oled_x = oled_x + (move_const * x_accel * x_dir/one_g);


        // set the new y position
        //we multiply by -1 to invert the direction, which makes the accelerometer tilting more intuitive.

        oled_y = oled_y + (move_const * y_accel * y_dir /one_g);


        //Message("Past position update\n");
        //bounds checking

        if ((oled_x * oled_x) + (oled_y * oled_y) > (640 * 640)){
            float angle = atan2(oled_y, oled_x);    //get angle of top left corner of oled, wrt 0,0 of the star map
            oled_x = 640 * cos(angle);              // set top left corner to same angle, but within bounds of the star map
            oled_y = 640 * sin(angle);
        }


        int i;
        //Report("\n%d\n", star_array_ind);
        for (i = 0; i < star_array_ind; i++){
            //Message("in for loop");
            //Report("index i: %d\n", i);
            //printf("%s\n",star_array[i].name);
            float temp_star_x = star_array[i].x;
            float temp_star_y = star_array[i].y;
            // bounds checking; render if within OLED screen bounds
            //Report("%f, %f", temp_star_x, temp_star_y);
            //Report("%f\n",temp_star_x > oled_x);

            if (temp_star_x > oled_x && temp_star_x < oled_x + WIDTH
               && temp_star_y > oled_y && temp_star_y < oled_y + HEIGHT
            ){
                // convert star map coords into OLED coords
                //Report("%f\n", oled_x);
                //Report("%f\n", oled_y);
                int screen_star_x = temp_star_x - oled_x;
                int screen_star_y = temp_star_y - oled_y;

                prev_star_screen_x[prev_screen_ind] = screen_star_x;
                prev_star_screen_y[prev_screen_ind] = screen_star_y;
                prev_screen_ind++;
                fillCircle(screen_star_x, screen_star_y, star_array[i].magnitude, WHITE);

            }
        }

//        if (x_accel > 5 || y_accel > 5){

            int j = 0;
            //Report("prev stars count: %d\n", prev_screen_ind);

            if (x_accel > 5 || y_accel > 5){
            for (j = 0; j < prev_screen_ind; j++){
                fillCircle(prev_star_screen_x[j], prev_star_screen_y[j], 1, GRAY);

            }

            for (j=0; j < prev_prev_screen_ind; j++){
                fillCircle(prev_prev_star_screen_x[j], prev_prev_star_screen_y[j], 1, BLACK);
            }
            }
            prev_prev_screen_ind = prev_screen_ind;
            memcpy(prev_prev_star_screen_x, prev_star_screen_x, prev_screen_ind * sizeof(int));
            memcpy(prev_prev_star_screen_y, prev_star_screen_y, prev_screen_ind * sizeof(int));
            prev_screen_ind = 0;
            //sleep(1);

//        }



        i = 0;



        if (buffer_ind >= 32){
            buffer_ind = 0;

            //int i;

            int first_set_buffer[8];
            int second_set_buffer[8];
            int third_set_buffer[8];
            int fourth_set_buffer[8];

            memcpy(&first_set_buffer, &buffer[0], sizeof(int) * 8);
            memcpy(&second_set_buffer, &buffer[8], sizeof(int) * 8);
            memcpy(&third_set_buffer, &buffer[16], sizeof(int) * 8);
            memcpy(&fourth_set_buffer, &buffer[24], sizeof(int) * 8);

            int recv_key = checkData(third_set_buffer, fourth_set_buffer, first_set_buffer, second_set_buffer);

            Report("%d\n\r", recv_key);


            if (recv_key == -1){
                Message("Noisy signal, data parity incorrect. Discarding value\n\r");
                continue;
            }
            if (recv_key == -2){
                Message("Noisy signal, address parity incorrect. Discarding value\n\r");
                continue;
            }

            if (recv_key == 12){//delete
//                send_buffer[send_index] = ' ';
//                send_index--;
//                for (i = 0; i < send_index; i++){
//                    Report("%c",send_buffer[i]);
//                }
//                continue;

            }
            else if ((third_set_buffer[4] == 1 && third_set_buffer[0] == 0)){//mute
                // format and send out a POST request
                format_and_send(lRetVal);
                send_index = 0;
                continue;
           }

        }

    }

}


void format_and_send(int iTLSSockID){
    char* formatted_send_str_first = "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"var\" :\"";


    char* formatted_send_str_second = "\"\r\n"                              \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n";

    // construct the request statement body
    char* final_formatted_str = calloc(200+MAX_STRING_LENGTH, sizeof(char));
    char* send_string = calloc(MAX_STRING_LENGTH, sizeof(char));
    strcat(final_formatted_str, formatted_send_str_first);
    int i = 0;
    for (i = 0; i < send_index; i++){
        Report("%c\n", send_buffer[i]);
        char* temp = calloc(2, sizeof(char));
        snprintf(temp, 2, "%c", send_buffer[i]);
        strcat(final_formatted_str, temp);


    }

    send_buffer[send_index]= '\0';
    Report("send_string: %s", send_buffer);
    strcat(final_formatted_str, send_buffer);
    strcat(final_formatted_str, formatted_send_str_second);
    //send_buffer[send_index+1] = '\0';
    Report("string: %s\n\r", final_formatted_str);

    Report("Send Index: %d\n\r", send_index);
    // send out a POST request
    http_post_msg(iTLSSockID, final_formatted_str);
    free(final_formatted_str);
    return;

}

static int http_post_msg(int iTLSSockID, char* data){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(data);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, data);
    pcBufHeaders += strlen(data);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static int http_get(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");


    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}





static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA1);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}
