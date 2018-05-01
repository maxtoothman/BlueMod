/**************************************************************************
 *
 * $Header: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/sto_testmode.h,v 1.1 2016/07/25 10:14:32 mj Exp $
 *
 * File:        $RCSfile: sto_testmode.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/sto_testmode.h,v $
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
 * Definitions for TESTMODE for BlueMod+S42 Development Kit
 *
 * ------------------------------------------------------------------------
 *
 * $Log: sto_testmode.h,v $
 * Revision 1.1  2016/07/25 10:14:32  mj
 * issue #0014746
 * Init. revision.
 *
 *
 **************************************************************************/
 

#include "boards.h"
#include "nrf.h"


uint32_t testmode_init( void);
uint32_t testmode_check(void);
uint32_t testmode_close(void);
