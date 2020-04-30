/**************************************************************************************************
  Filename:       OnBoard.h
  Revised:        $Date: 2012-04-18 14:06:48 -0700 (Wed, 18 Apr 2012) $
  Revision:       $Revision: 30252 $

  Description:    Defines stuff for EVALuation boards
  Notes:          This file targets the Chipcon CC2530/31


  Copyright 2005-2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ?S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef ONBOARD_H
#define ONBOARD_H


#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

// #include "OSAL.h"
#include "stddef.h"
#include <stdint.h>

#define 	OB_COLD  		0
#define 	OB_WARM  		1
#define 	OB_READY 		2


#define    CALIBRATION        	true         //calibration
#define    RF_MODE            	0

#define  	WK_MODE_CCT        	0x01       
#define     WK_MODE_GEL        	0x02   
#define     WK_MODE_EFF        	0x03   


#define     EFF_MODE_IDLE        	0x00  
#define     EFF_MODE_OFF        	0x01   

#define     EFF_MODE_STROBE        	0x02   
#define    	EFF_MODE_PAPARAZZI     	0x03      //eff_paparazzi
#define    	EFF_MODE_SHORTOUT     	0x04      //
#define     EFF_MODE_FIRE        	0x05   





/* */
typedef enum
{    
  DMX_SEND_OUT,
  DMX_RECEIVER,
} DMXDataDir_t;

/* */
typedef enum
{    
  INS_LED_MASTER,
  INS_LED_SLAVE,
  INS_LED_RF_OPEN,
  INS_LED_RF_CLOSE,
} sysINSLed_t;



/* 语言类型*/
#define    SYS_LANG_TYPE_EN          0x01
#define    SYS_LANG_TYPE_CN          0x02

/* 设备的主从 */
#define    	SYS_DEV_MASTER         	0x01		//主机
#define    	SYS_DEV_SLAVE         	0x02		//从机
#define		SYS_DEV_STANDBY			0x04    	//待机，既不是主从，为了不受DMX控制，而DMX信号可以通过
/* the size of system data */
#define SYSDATA_SIZE       sizeof(sys_data)

// extern sys_data sysdata;

extern uint16_t        logo_displayTime;



/*********************************************************************
 * FUNCTIONS
 */
/*
  * Initialize the Peripherals
  *    level: 0=cold, 1=warm, 2=ready
  */
extern void InitBoard ( uint8_t level );

/*
 * Get elapsed timer clock counts
 */
// extern uint32_t TimerElapsed ( void );

/*
 * Register for all key events
 */
extern uint8_t RegisterForKeys ( uint8_t task_id );

/* Keypad Control Functions */

/*
 * Send "Key Pressed" message to application
 */
extern uint8_t OnBoard_SendKeys( uint8_t keys, uint8_t shift );

/* LCD Emulation/Control Functions */
/*
 * Convert an interger to an ascii string
 */
extern void _itoa( uint16_t num, uint8_t *buf, uint8_t radix );


extern void Dimmer( uint8_t lvl );

// void sys_parameterInit(sys_data* pdec);

void DMX_dataDirectionSet(DMXDataDir_t dir);

void SYS_INSLedSet(sysINSLed_t sta);



/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif


