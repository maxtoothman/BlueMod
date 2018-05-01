/**************************************************************************
 *
 * $Header: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/common/sto_util.c,v 1.1 2016/07/25 10:14:12 mj Exp $
 *
 * File:        $RCSfile: sto_util.c,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/common/sto_util.c,v $
 * Revision:    $Revision: 1.1 $
 * Date:        $Date: 2016/07/25 10:14:12 $
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
 * Utilities for BlueMod+S42 Development Kit
 *
 * ------------------------------------------------------------------------
 *
 * $Log: sto_util.c,v $
 * Revision 1.1  2016/07/25 10:14:12  mj
 * issue #0014746
 * Init. revision.
 *
 *
 **************************************************************************/
 

#include "sto_util.h"

#include "nrf.h"
#include "ble.h"
#include "ble_hci.h"
#include "app_error.h"


uint16_t btStringToBd(char * source, uint8_t * dest)
{
   uint16_t endLoop = 12;
   bool lower   = false;
   uint16_t loop;
   uint8_t c;

   if ( source == 0)
    {
     source = (char *) "00:00:00:00:00:00";
    }

   for(loop=0;loop<endLoop;loop++)
   {
      c = (uint8_t) TOUPPER(source[loop]);

      if(((c>='0')&&(c<='9'))||((c>='A')&&(c<='F')))
      {  if(lower) /* ODD  ==> lower nibble   */
         {  if(dest)
            {  *dest |= ASCIIHexToNibble(c);
               dest++;
            }
            lower=false;
         } else     /* EVEN ==> upper nibble   */
         {  if(dest)
            {  *dest  = ASCIIHexToNibble(c)<<4;
            }
            lower=true;
         }
      } else
      {  if(c==':') /* official byte seperator */
         {  endLoop++;
         } else
         {  return 0; /* error */
         }
      }
   }
   return endLoop;  /* OK ==> return handled char's  */
}

 
