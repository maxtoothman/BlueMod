 /**************************************************************************
 *
 * $Header: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/sto_util.h,v 1.1 2016/07/25 10:14:32 mj Exp $
 *
 * File:        $RCSfile: sto_util.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/sto_util.h,v $
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
 * Defintions for utilities for BlueMod+S42 Development Kit
 *
 * ------------------------------------------------------------------------
 *
 * $Log: sto_util.h,v $
 * Revision 1.1  2016/07/25 10:14:32  mj
 * issue #0014746
 * Init. revision.
 *
 *
 **************************************************************************/
 

#ifndef STO_UTIL_H__
#define STO_UTIL_H__

#include "nrf.h"
#include "ble.h"

uint16_t btStringToBd(char * source, uint8_t * dest);



#define TOUPPER(c) ((( c >= 'a' ) && ( c <= 'z' )) ? ( c - 'a' + 'A' ) : c )

#define ASCIIHexToNibble( c ) (( c <= '9' )?((char)(c-'0')):((char)(c-'A')+0x0A))


/**
 * @brief Function for disable the given GPIO pin number 
 *
 * @param pin_number specifies the pin number of gpio pin numbers to be disbaled (allowed values 0-30)
 *
 */
static __INLINE void nrf_gpio_cfg_disable(uint32_t pin_number)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}


#endif
