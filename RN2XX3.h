/* MIT License
//
// Copyright (c) 2017 Beach Cities Software, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// ___________________     _________       _____  __                                 
// \______   \_   ___ \   /   _____/ _____/ ____\/  |___  _  _______ _______   ____  
//  |    |  _/    \  \/   \_____  \ /  _ \   __\\   __\ \/ \/ /\__  \\_  __ \_/ __ \ 
//  |    |   \     \____  /        (  <_> )  |   |  |  \     /  / __ \|  | \/\  ___/ 
//  |______  /\______  / /_______  /\____/|__|   |__|   \/\_/  (____  /__|    \___  >
//         \/        \/          \/                                 \/            \/ 
*/


// RN2XX3.h
#ifndef _RN2XX3_H
#define	_RN2XX3_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdbool.h>        // bool data type

#define RN2XX3_UART_PORT   "/dev/ttyAMA0"

// #define RECEIVE_DELAY1                              100UL
// #define RECEIVE_DELAY2                              100UL
// #define JOIN_ACCEPT_DELAY1                          100UL
// #define JOIN_ACCEPT_DELAY2                          100UL
// #define MAX_FCNT_GAP                                10
// #define ADR_ACK_LIMIT                               3
// #define ADR_ACK_DELAY                               100
// #define ACK_TIMEOUT                                 2000

// typedef enum
// {
//     OK                                       = 0,
//     NETWORK_NOT_JOINED                          ,
//     MAC_STATE_NOT_READY_FOR_TRANSMISSION        ,
//     INVALID_PARAMETER                           ,
//     KEYS_NOT_INITIALIZED                        ,
//     SILENT_IMMEDIATELY_ACTIVE                   ,
//     FRAME_COUNTER_ERROR_REJOIN_NEEDED           ,
//     INVALID_BUFFER_LENGTH                       ,
//     MAC_PAUSED                                  ,
//     NO_CHANNELS_FOUND                           ,
// } LorawanError_t;     

// typedef union
// {
//     uint32_t value;
//     struct
//     {  unsigned networkJoined :1;                //if set, the network is joined
//        unsigned macState :3;                      //determines the state of trasmission (rx window open, between tx and rx, etc)
//        unsigned automaticReply :1;               //if set, ACK and uplink packets sent due to  FPending will be sent immediately
//        unsigned adr :1;                           //if set, adaptive data rate is requested by server or application
//        unsigned silentImmediately :1;             //if set, the Mac command duty cycle request was received
//        unsigned macPause :1;                      //if set, the mac Pause function was called. LoRa modulation is not possible
//        unsigned rxDone :1;                        //if set, data is ready for reception
//        unsigned linkCheck :1;                     //if set, linkCheck mechanism is enabled
//        unsigned channelsModified :1;             //if set, new channels are added via CFList or NewChannelRequest command or enabled/disabled via Link Adr command
//        unsigned txPowerModified :1;              //if set, the txPower was modified via Link Adr command
//        unsigned nbRepModified :1;                 //if set, the number of repetitions for unconfirmed frames has been modified
//        unsigned prescalerModified :1;             //if set, the prescaler has changed via duty cycle request
//        unsigned secondReceiveWindowModified :1;   //if set, the second receive window parameters have changed
//        unsigned rxTimingSetup :1;                 //if set, the delay between the end of the TX uplink and the opening of the first reception slot has changed
//        unsigned rejoinNeeded :1;                 //if set, the device must be rejoined as a frame counter issue happened
//     };
// } LorawanStatus_t;

/**
* Initialize the serial connection 
*/
void RN2XX3_Init();

/**
* Returns the file pointer, to the open UART serial, or 0 if uninitialized or failure.
* Check errno if it returns 0 for errors generated by system call open.
* @return The file pointer, to the open UART serial, or 0 if uninitialized or failure.
*/
int RN2XX3_GetSerial();


/**
* Sets the timeout for waiting for UART rx, default is: 500ms
* @param int timeoutMs
*/
void RN2XX3_SetSerialRxTimeout(const int timeoutMs);


/**
* Closes the UART connection to RN2XX3 module
* @return int - returns 0 on success, 1 on failure, -1 if already closed
*/
int RN2XX3_Close();

/**
* Executes an arbitrary command against the RN2XX3 command API and returns the response.
* @param cmd - The command to send over UART
* @returns char* - the output from the command execution, or NULL on failure
*/
const char *RN2XX3_ExecCmd(const char *cmd);


/*=====================
// SYS COMMANDS
//===================*/


/**
* Sleeps with the RN2XX3 modules sys sleep function
* @param intervalMs - Sleep time in milliseconds
*/
bool RN2XX3_SysSleep(const int intervalMs);

/**
* Calls sys get ver and returns the version of the RN2XX3 module's firmware
* @returns char* - Returns the version of the RN2XX3 module, or NULL on failure
*/
const char* RN2XX3_GetSysVersion();


/*=====================
// MAC COMMANDS
//===================*/



/*=====================
// RADIO COMMANDS
//===================*/


#ifdef	__cplusplus
}
#endif

#endif	/* _RN2XX3_H */