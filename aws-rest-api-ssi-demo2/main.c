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


// Standard includes
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>


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

float latitude = 38.54;
float longitude = -121.74;
int years = 2024;
int months = 6;
int days = 4;
int hours = 23;
int minutes = 30;
int sign = 1;

#define PI 3.14159265358979323846
#define CONTROL_MODE 0  // control mode 0 for accelerometer control, 1 for remote control

//communication with OLED
#define MOSI_Pin Pin_07
#define CLK_Pin Pin_05
#define DC_Pin 0x80 // Pin 45
#define Reset_Pin 0x10 // Pin 18
#define OC_Pin 0x2 // Pin 8
#define SPI_IF_BIT_RATE 100000

//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                2    /* Current Date */
#define MONTH               6     /* Month 1-12 */
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
// char send_buffer[128][128];   //buffer for messages to be sent out
char send_buffer_hex[4100];
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
    systick_elapsed = 0;

}

//pin 63 info
#define IR_GPIO_PORT GPIOA1_BASE
#define IR_GPIO_PIN 0x1
volatile int systick_delta = 0; //formerly ulsystick_delta_us

// GPIO handler

static void GPIOA0IntHandler(void) {

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

static int set_time() {
//    long retVal;
//
//    g_time.tm_day = DATE;
//    g_time.tm_mon = MONTH;
//    g_time.tm_year = YEAR;
//    g_time.tm_sec = HOUR;
//    g_time.tm_hour = MINUTE;
//    g_time.tm_min = SECOND;
//
//    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
//                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
//                          sizeof(SlDateTime),(unsigned char *)(&g_time));
//
//    ASSERT_ON_ERROR(retVal);
//    return SUCCESS;
}

struct Star{
    char* name;
   float ra[3];
   float dec[3];
    float x;
    float y;
    int magnitude;
};


unsigned char accel_reg_buffer[256] = {0};
// Based on ProcessReadRegCommand(), except it takes three parameters instead of parsing a string
unsigned char newReadReg(unsigned char ucDevAddr, unsigned char ucRegOffset, unsigned char ucRdLen){
        // Write the register address to be read from.
        // Stop bit implicitly assumed to be 0.
        RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));

        // read acceleration data into accel_reg_buffer, which is a global array. Only need to read one byte
        RET_IF_ERR(I2C_IF_Read(ucDevAddr, &accel_reg_buffer[0], ucRdLen));

        return 0;

}

long lRetVal = -1;
int prev_val = -1;
void start_up() {
    lRetVal = -1;
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


     init_OLED_board();


     init_aws();

    Report("period length: %d\n\r",SYSTICK_RELOAD_VAL );


}

void init_OLED_board() {
    Message("Initializing OLED...\n\r");
    Adafruit_Init();

    Message("Done initializing OLED\n\r");

    fillScreen(BLACK);
}

void init_aws() {
    // code for connecting to aws
    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    // Connect the CC3200 to the local access point
    Message("Before connect to access point");
    lRetVal = connectToAccessPoint();
    Message("after connect to access point");
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    // Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
    // send out POST and GET request before accepting user input
    http_post(lRetVal);

}

int star_array_ind = 0;
struct Star star_array[200];

void read_star_map_file() {


    char* star_data[]  = {
"Name,RA hour,RA minute,RA second,DEC degree,DEC arcminute,DEC arcsecond",
"Polaris,2,31,49.08,89,15,50.8",
"Kochab,14,50,42.33,74,9,19.8",
"Pherkad (Gamma Ursae Minoris),15,20,43.71,71,50,2.5",
"Epsilon ursae minoris,16,45,58.24,82,2,14.1",
"Yildun (delta ursae minoris),17,32,13,86,35,11.2",
"zeta ursae minoris,15,44,3.52,77,47,40.2",
"eta ursae minoris,16,17,30.29,75,45,19.2",
"Alioth,12,54,1.75,55,57,35.4",
"Dubhe,11,3,43.67,61,45,3.7",
"Alkaid,13,47,32.44,49,18,47.8",
"Mizar,13,23,55.54,54,55,31.3",
"Merak,11,1,50.48,56,22,56.7",
"Phecda,11,53,49.85,53,41,41.1",
"Megrez,12,15,25.56,57,1,57.4",
"Alderamin (alpha),21,18,34.58,62,35,7.6",
"Errai (gamma),23,39,20.98,77,37,55.1",
"Alfirk (beta),21,28,39.58,70,33,38.5",
"Zeta cephei,22,10,51.26,58,12,4.5",
"Al kidr (eta),20,45,17.27,61,50,12.5",
"iota cephei,22,49,40.91,66,12,2.6",
"Schedar (alpha),0,40,30.39,56,32,14.7",
"Caph (beta),0,9,10.09,59,9,0.8",
"Tsih (gamma),0,56,42.5,60,43,0.3",
"Ruchbah (delta),1,25,48.6,60,14,7.5",
"Segin (epsilon),1,54,23.68,63,40,12.5",
"Arcturus (alpha),14,15,40.35,19,11,14.2",
"Epsilon bootis,14,44,59.22,27,4,27.2",
"eta bootis,13,54,41.12,18,23,54.9",
"gamma bootis,14,32,4.76,38,18,28.4",
"delta bootis,15,15,30.1,33,18,54.4",
"beta bootis,15,1,56.79,40,23,26.3",
"Rho bootis,14,31,49.86,30,22,16.1",
"pi bootis,14,40,43.56,16,25,5.9",
"zeta bootis,14,41,8.9,13,43,42",


};

   char curr_line[150];
   char delim[2] = ",";
   char* val;
   //Referenced https://www.geeksforgeeks.org/strtok-strtok_r-functions-c-examples/
        int a;
        for (a = 1; a < 35; a++){
           val = strtok(star_data[a], delim);
           star_array[star_array_ind].name = val;

           Report("%s\n",val);


           // read in right ascension and declination data

           int j = 0;
           for (j = 0; j < 3; j++){
               val = strtok(NULL, delim); //keep performing strtok from after the current token
               Report("RA: %s", val);
               star_array[star_array_ind].ra[j] = atof(val);

           }
           j = 0;
           while(val != 0){
               val = strtok(NULL, delim);
               star_array[star_array_ind].dec[j] = atof(val);
               j++;
           }
           val = strtok(NULL, delim);
            star_array[star_array_ind].magnitude = atoi(val);
            Report("name: %s, RA: %fHR %fMIN %fS, Dec: %f %f' %f'', magnitude: %d", star_array[star_array_ind].name, star_array[star_array_ind].ra[0], star_array[star_array_ind].ra[1], star_array[star_array_ind].ra[2], star_array[star_array_ind].dec[0], star_array[star_array_ind].dec[1], star_array[star_array_ind].dec[2], star_array[star_array_ind].magnitude);


            // Do calculations for converting right ascension and declination to local measurements altitude and azimuth

            struct tm j2000 = {0};    // initialize to noon on Jan 1st of 2000
            j2000.tm_sec = 0;
            j2000.tm_min = 0;
            j2000.tm_hour = 12;
            j2000.tm_mday = 1;
            j2000.tm_mon = 0;
            j2000.tm_year = 100;


            struct tm current = {0};

            current.tm_sec = 0;
            current.tm_min = minutes;
            current.tm_hour = hours + 7;    //assume Davis, CA time. Convert to UTC
            current.tm_mday = days;
            current.tm_mon = months - 1;
            current.tm_year = years - 1900;

            float ut_hours;
            if (hours + 7 < 24){
                ut_hours = hours + 7;    //assume Davis, CA time. Convert to UTC
            }
            else{
                ut_hours = hours + 7 - 24;
            }

            //https://stackoverflow.com/questions/28880849/convert-struct-tm-to-time-t
            time_t j2000_raw = mktime(&j2000);
            time_t current_raw = mktime(&current);

            double days_since_j2000 = difftime(current_raw, j2000_raw)/86400;

            double local_siderial_time = 100.46 + 0.985647 * days_since_j2000 + longitude + (15 * (ut_hours + minutes/60));
            if (local_siderial_time < 0){
                local_siderial_time +=360;
            }

            double ra_in_degrees =  15 * (star_array[star_array_ind].ra[0]*3600 + star_array[star_array_ind].ra[1]*60 + star_array[star_array_ind].ra[2])/3600;
            double dec_in_degrees = star_array[star_array_ind].dec[0] + star_array[star_array_ind].dec[1]/60 + star_array[star_array_ind].dec[2]/3600;
            double hour_angle = local_siderial_time - ra_in_degrees;
            if (hour_angle < 0){
                hour_angle += 360;
            }


            double dec_in_rad = dec_in_degrees * PI/180;
            double ra_in_rad = ra_in_degrees * PI/180;
            double latitude_in_rad = latitude * PI/180;
            double hour_angle_in_rad = hour_angle * PI/180;


            double altitude = asin(sin(dec_in_rad)*sin(latitude_in_rad) + cos(dec_in_rad)*cos(latitude_in_rad)*cos(hour_angle_in_rad));


            double azimuth = acos( (sin(dec_in_rad) - sin(altitude)*sin(latitude_in_rad))/(cos(altitude)*cos(latitude_in_rad)) );

            if (sin(hour_angle_in_rad) > 0){
                azimuth = 2 * PI - azimuth;
            }

            // convert to x and y
            // do stereographic projection from ra and dec to flat polar coords
            // ra -> azimuth, dec -> altitude
            float projection_theta = azimuth;
            float projection_rad = 2 * tan(PI/4 - altitude/2 );
            star_array[star_array_ind].x = cos(projection_theta) * 128 * projection_rad;
            star_array[star_array_ind].y = sin(projection_theta) * 128 * projection_rad;
            Report("x: %f, y: %f",star_array[star_array_ind].x,  star_array[star_array_ind].y);

           star_array_ind++;
       }
       Message("Done reading in data\n\r");

}


char calibration_buffer[5];
int calibration_buffer_ind = 0;

int x_cursor = 0;
int y_cursor = 0;
int await_remote_input(){
    while(1){

        if (buffer_ind >= 32){
            buffer_ind = 0;


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
                calibration_buffer_ind--;
                x_cursor -= 8;
                calibration_buffer[calibration_buffer_ind] = '\0';

                if (x_cursor < 0){
                    x_cursor = WIDTH - 8;
                    y_cursor -= 12;
                }

            }
            else if ((third_set_buffer[4] == 1 && third_set_buffer[0] == 0)){//mute
                int arrSize = sizeof(calibration_buffer) / sizeof(calibration_buffer[0]);
                Report("ArrSize: %d\n\r", arrSize);
                int i = 0;
                Message("Calibration_buffer: ");
                for (i = 0; i < arrSize; i++) {
                    Report("%c", calibration_buffer[i]);
                }
                Message("\n\r");
                calibration_buffer_ind = 0;

                int returnNum = atoi(calibration_buffer);
                memset(calibration_buffer, '\0', sizeof(calibration_buffer));
                Report("Num: %d\n\r", returnNum);
                return returnNum;
            }
            else{   //a number 0-9
                Report("CurrCalIdx: %d\n\r", calibration_buffer_ind);
                calibration_buffer[calibration_buffer_ind] = '0' + recv_key;
                Report("recv_key: %c \n\r", '0' + recv_key);
                x_cursor = x_cursor + 9;
                drawChar(x_cursor, y_cursor, '0' + recv_key, WHITE, BLACK, 1);
                calibration_buffer_ind++;
            }

        }
    }
}




void draw_string(char* str){
    int i;
    for (i = 0; i < strlen(str); i++){
        drawChar(x_cursor, y_cursor, str[i], WHITE, BLACK, 1);
        x_cursor += 6;
            //cursor_y += 16;
            if (x_cursor >= WIDTH){
                x_cursor = 0;
                y_cursor += 12;
            }
    }


}






char valToDisplay[6];


void convertIntToStr(int val) {
    int displayIdx = 0;
    int valCopy = val;
    memset(valToDisplay, '\0', sizeof(valToDisplay));
    if (val < 0) {
        char negValToDisplay[5];
        valToDisplay[0] = '-';
        valCopy = valCopy  * (-1);
        sprintf(negValToDisplay, "%d", valCopy);
        strcat(valToDisplay, negValToDisplay);
    } else {
        sprintf(valToDisplay, "%d", val);
    }
}

void displayAllCalibrationData() {

    Message("\n\r");
    Report("Years: %d\n\r", years);
    Report("Months: %d\n\r", months);
    Report("Days: %d\n\r",days);
    Report("Hours: %d\n\r", hours);
    Report("minutes: %d\n\r",minutes);
    Report("longitude: %d\n\r", longitude);
    Report("latitude: %d\n\r", latitude);
}


void calibrate_location() {
    char* time_msg1 = "Enter current time";
    char* time_msg2 = "(in UTC)";
    char* time_msg3 = "Hit MUTE to confirm";
    char* time_msg4 = "Hit LAST to delete";

    char* year_msg = "Years: ";
    char* month_msg = "Months: ";
    char* day_msg = "Days: ";
    char* hour_msg = "Hours: ";
    char* minute_msg = "Minutes: ";

    char* location_msg1 = "Enter location.";
    char* location_msg2 = "1 for +, 0 for -";
    char* lat_msg1 = "Latitude +/-: ";
    char* lat_msg2 = "Latitude val: ";
    char* long_msg1 = "Longitude +/-: ";
    char* long_msg2 = "Longitude val: ";

    goTo(0,0);
    draw_string(time_msg1);
    x_cursor = 0;
    y_cursor += 12;
    draw_string(time_msg2);
    x_cursor = 0;
    y_cursor += 12;
    draw_string(time_msg3);
    x_cursor = 0;
    y_cursor += 12;
    draw_string(time_msg4);
    x_cursor = 0;
    y_cursor += 24;

    draw_string(year_msg);
    years = await_remote_input();
    x_cursor = 0;
    y_cursor += 12;
    draw_string(month_msg);
    months = await_remote_input();
    x_cursor = 0;
    y_cursor += 12;
    draw_string(day_msg);
    days = await_remote_input();
    x_cursor = 0;
    y_cursor += 12;
    draw_string(hour_msg);
    hours = await_remote_input();
    x_cursor = 0;
    y_cursor += 12;
    draw_string(minute_msg);
    minutes = await_remote_input();


    fillScreen(BLACK);
    x_cursor = 0;
    y_cursor = 0;
    draw_string(location_msg1);
    x_cursor = 0;
    y_cursor += 12;
    draw_string(location_msg2);
    x_cursor = 0;
    y_cursor += 24;

    draw_string(lat_msg1);
    sign = await_remote_input();
    x_cursor = 0;
    y_cursor += 12;
    draw_string(lat_msg2);
    latitude = await_remote_input();
    if (sign == 0) {
        latitude = latitude * (-1);
    }
    x_cursor = 0;
    y_cursor += 12;

    draw_string(long_msg1);
    sign = await_remote_input();
    x_cursor = 0;
    y_cursor += 12;
    draw_string(long_msg2);
    longitude = await_remote_input();
    if (sign == 0) {
        longitude = longitude * (-1);
    }
    fillScreen(BLACK);
    displayAllCalibrationData();
    fillScreen(BLACK);
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

    int prev_star_screen_x[100];
    int prev_star_screen_y[100];
    int prev_screen_ind = 0;
    int prev_prev_star_screen_x[100];
    int prev_prev_star_screen_y[100];
    int sorted_prev_star_screen_x[100];
    int sorted_prev_star_screen_y[100];
    int prev_prev_screen_ind = 0;

int find_min_ind(int arr[], int arrSize) {
    int i = 0;
    int currMinVal = 200;
    int currMinIdx = 0;
    for (i = 0; i < arrSize; i++) {
        if(arr[i] < currMinVal) {
            currMinVal = arr[i];
            currMinIdx = i;
        }
    }
    return currMinIdx;
}

void sort_prev_star_screen_xy() {
    int copy_prev_star_screen_x[prev_screen_ind - 1];
    int copy_prev_star_screen_y[prev_screen_ind - 1];
    int l;
    Message("Copy: \n\r");
    for (l = 0; l < prev_screen_ind; l++){
        Report("y: %d, x: %d\n\r", prev_star_screen_y[l], prev_star_screen_x[l]);
        copy_prev_star_screen_x[l] = prev_star_screen_x[l];
        copy_prev_star_screen_y[l] = prev_star_screen_y[l];
    }
    Message("\n\r");
    Message("Sorted: \n\r");
    for (l = 0; l < prev_screen_ind; l++) {
        int currMinIdx = find_min_ind(copy_prev_star_screen_y, prev_screen_ind);
        Report("CurrMinIdx: %d\n\r", currMinIdx);
        if(copy_prev_star_screen_y[currMinIdx] == 200) {
            return;
        }
        sorted_prev_star_screen_x[l] = copy_prev_star_screen_x[currMinIdx];
        sorted_prev_star_screen_y[l] = copy_prev_star_screen_y[currMinIdx];
        copy_prev_star_screen_y[currMinIdx] = 200;
    }
    return;
}

void populate_send_buffer_hex() {
    memset(send_buffer_hex, '0', sizeof(send_buffer_hex));
    int currY = 0;
    int currX = 0;
    int i = 0;
    int j = 0;
    int star_screen_idx = 0;
    int send_buffer_hex_idx = 0;

    int row[128];
    for (currY = 0; currY < 128; currY++) {

        memset(row, 0, sizeof(row));

        if(currY == sorted_prev_star_screen_y[star_screen_idx]) {
            while(currY == sorted_prev_star_screen_y[star_screen_idx]) {
                row[sorted_prev_star_screen_x[star_screen_idx]] = 1;
                star_screen_idx++;
            }

        }

        Message("\n\r");
        Message("Starting compression!!!");
        for(currX = 0; currX < 32; currX++) {

            char currBinary[4];
            int currHexChar = 0;
            char temp[2];

            for (i = 0; i < 4; i++) {
                currBinary[i] = row[currX * 4 + i];
                currHexChar = currHexChar+ (currBinary[i] * pow(2, i));

            }

            send_buffer_hex[send_buffer_hex_idx] = currHexChar + 48;
            send_buffer_hex_idx++;

        }

        Message("\n\r");
    }

    for (i = 0; i < 128; i++) {
        for (j = 0; j < 32; j++) {
            Report("%c", send_buffer_hex[i*32 + j]);
        }
        Message("\n\r");
    }
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
void main() {
    start_up();

    calibrate_location();

    read_star_map_file();


    // code from lab 3 that decodes the IR signal
    buffer_ind = 0;
     while(1){
         //read and set current value of x acceleration
         newReadReg((unsigned char)0x18, (unsigned char)0x3, (unsigned char)1);
         x_accel = accel_reg_buffer[0];

         //read and set current value of y acceleration
         newReadReg((unsigned char)0x18, (unsigned char)0x5, (unsigned char)1);
         y_accel = accel_reg_buffer[0];

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

         // set the new x position. Start with the current position, then decide how much to change the position based on accelerometer
         // measurement (magnitude and direction). Move_const changes the speed at which the ball moves.


         oled_x = 320 * x_accel * x_dir * -1 /one_g - 64;
         // set the new y position
         //we multiply by -1 to invert the direction, which makes the accelerometer tilting more intuitive.


         oled_y = 320 * y_accel * y_dir /one_g -64;


         int i;

         for (i = 0; i < star_array_ind; i++){

             float temp_star_x = star_array[i].x;
             float temp_star_y = star_array[i].y;
             // bounds checking; render if within OLED screen bounds

             if (temp_star_x > oled_x && temp_star_x < oled_x + WIDTH
                && temp_star_y > oled_y && temp_star_y < oled_y + HEIGHT
             ){
                 // convert star map coords into OLED coords

                 int screen_star_x = temp_star_x - oled_x;
                 int screen_star_y = temp_star_y - oled_y;

                 prev_star_screen_x[prev_screen_ind] = screen_star_x;
                 prev_star_screen_y[prev_screen_ind] = screen_star_y;
                 prev_screen_ind++;
                 fillCircle(screen_star_x, screen_star_y, star_array[i].magnitude, WHITE);
                 fillCircle(screen_star_x, screen_star_y, star_array[i].magnitude, GRAY);

             }
         }

         int j = 0;

         for (j=0; j < prev_prev_screen_ind; j++){
             fillCircle(prev_prev_star_screen_x[j], prev_prev_star_screen_y[j], 1, BLACK);
         }


         prev_prev_screen_ind = prev_screen_ind;
         memcpy(prev_prev_star_screen_x, prev_star_screen_x, prev_screen_ind * sizeof(int));
         memcpy(prev_prev_star_screen_y, prev_star_screen_y, prev_screen_ind * sizeof(int));




         i = 0;



         if (buffer_ind >= 32){
             buffer_ind = 0;



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

             }
             else if ((third_set_buffer[4] == 1 && third_set_buffer[0] == 0)){//mute
                 // format and send out a POST request


                 int l;
                 for (l = 0; l < prev_screen_ind; l++){
                     Report("y: %d, x: %d\n\r", prev_star_screen_y[l], prev_star_screen_x[l]);

                 }

                 sort_prev_star_screen_xy();
                 for (l = 0; l < prev_screen_ind; l++){
                     Report("y: %d, x: %d\n\r", sorted_prev_star_screen_y[l], sorted_prev_star_screen_x[l]);

                 }
                 Message("\n\r\n\r\n\r");
                 Message("Populating send buffer hex\n\r");
                 populate_send_buffer_hex();


                 format_and_send(lRetVal);
                 send_index = 0;
                 Message("Out of format and send");
                 continue;
            }

         }

         prev_screen_ind = 0;

     }

}

char convertBinToHex(char* bin) {
    int k = 0;
    int dec = 0;
    for (k = 0; k < 4; k++) {
        if(bin[k] == '1') {
            dec = dec + pow(2, 3 - k);
        }
    }
    char retVal = 'g';
    int temp = dec % 16;
    if (temp < 10) {
        retVal = temp + 48;
    } else {
        retVal = temp + 55;
    }
    return retVal;
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
    char* final_formatted_str = calloc(4100+MAX_STRING_LENGTH, sizeof(char));
    char* send_string = calloc(MAX_STRING_LENGTH, sizeof(char));
    strcat(final_formatted_str, formatted_send_str_first);
    int i = 0;
    for (i = 0; i < send_index; i++){
        Report("%c\n", send_buffer_hex[i]);
        char* temp = calloc(2, sizeof(char));
        snprintf(temp, 2, "%c", send_buffer_hex[i]);
        strcat(final_formatted_str, temp);


    }

    send_buffer_hex[4096]= '\0';
    Report("send_string: %s", send_buffer_hex);
    strcat(final_formatted_str, send_buffer_hex);
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

