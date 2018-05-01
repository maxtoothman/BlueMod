/**************************************************************************
 *
 * $Header: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/custom_board.h,v 1.1 2016/07/25 10:14:32 mj Exp $
 *
 * File:        $RCSfile: custom_board.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/custom_board.h,v $
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
 * Called by Nordics boards.h file. Used for BlueMod+S42 Development Kit
 *
 * ------------------------------------------------------------------------
 *
 * $Log: custom_board.h,v $
 * Revision 1.1  2016/07/25 10:14:32  mj
 * issue #0014746
 * Init. revision.
 *
 *
 *************************************************************************/

#if defined(CUSTOM_BOARD_BLUEMOD_S42_EVAL)
  #include "bluemod_s42_eval.h"
#else
#error "Module is not defined"
#endif

