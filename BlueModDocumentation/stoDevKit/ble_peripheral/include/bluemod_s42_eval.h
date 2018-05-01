/**************************************************************************
 *
 * $Header: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/bluemod_s42_eval.h,v 1.1 2016/07/25 10:14:32 mj Exp $
 *
 * File:        $RCSfile: bluemod_s42_eval.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/bluemod_s42_eval.h,v $
 * Revision:    $Revision: 1.1 $
 * Date:        $Date: 2016/07/25 10:14:32 $
 * Author:      $Author: mj $
 *
 * ------------------------------------------------------------------------
 * !MODULE      [  ]
 * ------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name:]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: mj $]
 *
 *          Copyright (c)           2016 Telit Wireless Solutions
 *                                  Mendelssohnstrasse 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          All Rights Reserved
 *
 * ------------------------------------------------------------------------
 * !DESCRIPTION
 *
 * Additional definitions for BlueMod+S42 Development Kit
 *
 * ------------------------------------------------------------------------
 *
 * $Log: bluemod_s42_eval.h,v $
 * Revision 1.1  2016/07/25 10:14:32  mj
 * issue #0014746
 * Init. revision.
 *
 *
 **************************************************************************/
 
#ifndef BLUEMOD_S42_EVAL_H
#define BLUEMOD_S42_EVAL_H

#include "bluemod_s42.h"


#define LEDS_NUMBER    2

// #define LED_START      
#define LED_1          GPIO2
#define LED_2          GPIO3
//#define LED_STOP       24

#define LEDS_LIST { LED_1, LED_2 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 1

//#define BUTTON_START   GPIO1
#define BUTTON_1       GPIO1
#define BUTTON_STOP    GPIO1
#define BUTTON_PULL    NRF_GPIO_PIN_NOPULL

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)

#define BUTTONS_MASK   (BSP_BUTTON_0_MASK)

#endif // BLUEMOD_S42_EVAL_H
