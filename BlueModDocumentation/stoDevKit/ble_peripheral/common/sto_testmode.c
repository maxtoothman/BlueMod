/**************************************************************************
 *
 * $Header: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/common/sto_testmode.c,v 1.1 2016/07/25 10:14:12 mj Exp $
 *
 * File:        $RCSfile: sto_testmode.c,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/common/sto_testmode.c,v $
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
 * TESTMODE routines for BlueMod+S42 Development Kit
 *
 * ------------------------------------------------------------------------
 *
 * $Log: sto_testmode.c,v $
 * Revision 1.1  2016/07/25 10:14:12  mj
 * issue #0014746
 * Init. revision.
 *
 *
 **************************************************************************/
 
#include <ctype.h>
#include "boards.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "app_util.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "ble_flash.h"

#include "bsp.h"

#include "sto_testmode.h"
#include "sto_dtmmode.h"
#include "sto_util.h"


typedef enum
{
    TESTMODE_OFF,                                                            
    TESTMODE_ON,
		TESTMODE_IDLE,
  	DTMMODE_ON,
    FLASHMODE_ON,
		
    TESTMODE_WAIT_CLOSE,                                                        
} testmode_states_t;

typedef enum
{
    CLIMODE_WAIT_CMD,                                                            
    CLIMODE_PROCESS_CMD,                                                            
} climode_states_t;

typedef enum
{
		ON_TESTMODE_HIGH_BOOT0_HIGH,
		ON_TESTMODE_LOW_BOOT0_HIGH,
		ON_TESTMODE_HIGH_BOOT0_LOW,
		ON_TESTMODE_LOW_BOOT0_LOW,
} testmode_state_events_t;

static uint32_t testmode_uart_write( const uint8_t * p_buffer, uint32_t length);

#define PARSER_UKNOWN_CMD "\r\n-ERR 002;unknown cmd"
#define PARSER_PROMPT     "\r\n#"
#define PARSER_PROMPT_LEN 3

#define UART_MaxBufferSize	64

static uint8_t *m_uartRxBuffer;
static uint8_t *m_uartTxBuffer;


static volatile testmode_states_t  m_current_state = TESTMODE_OFF;              /**< State of the state machine. */

typedef struct
{
  uint8_t                      pin_no;
  nrf_drv_gpiote_in_config_t   pin_cfg;
} testmode_pin_cfg_t;

static const testmode_pin_cfg_t testmode_pin_cfg[2] =
{
  {
    TESTMODE,
    {
      NRF_GPIOTE_POLARITY_TOGGLE,
      NRF_GPIO_PIN_PULLUP,
      false,    /* is_watcher  */
      false     /* hi_accuracy */  
    }
  },
  {
    BOOT0,
    {
      NRF_GPIOTE_POLARITY_TOGGLE,
      NRF_GPIO_PIN_PULLDOWN,
      false,
      false
    }
  }
};

static uint32_t               m_pin_io_mode_mask;   
#define TESTMODE_CMDLINE_SIZE		80
static uint8_t *               inputCmdLine;
static volatile uint8_t				inputCmdLinePos;

static uint8_t * radio_packet;
static uint8_t radio_mode      = RADIO_MODE_MODE_Ble_1Mbit;
static uint8_t radio_txpower   = RADIO_TXPOWER_TXPOWER_0dBm;
static uint8_t radio_channel   = 39;
static uint8_t radio_dcdc		   = POWER_DCDCEN_DCDCEN_Enabled;

typedef void (* PfCmdHandler)(uint8_t * cmdline);

typedef struct _atCmdTag
{
  char  cmd[12];
  PfCmdHandler cmdHandler;
} TestModeCmdDefinition;

static void testmodeHandleVER(uint8_t * cmdline);
static void testmodeHandleVERB(uint8_t * cmdline);
static void testmodeHandleBOAD(uint8_t * cmdline);
static void testmodeHandleDEVAD(uint8_t * cmdline);
static void testmodeHandleHELP(uint8_t * cmdline);
static void testmodeHandleTXPWR(uint8_t * cmdline);
static void testmodeHandleRFOFF(uint8_t * cmdline);
static void testmodeHandleTXCW(uint8_t * cmdline);
static void testmodeHandleTXMOD(uint8_t * cmdline);
static void testmodeHandleRXON(uint8_t * cmdline);
static void testmodeHandleRFCHAN(uint8_t * cmdline);
static void testmodeHandleDD(uint8_t * cmdline);
static void testmodeHandleDW(uint8_t * cmdline);
static void testmodeHandleDB(uint8_t * cmdline);
static void testmodeHandleWD(uint8_t * cmdline);
static void testmodeHandleWW(uint8_t * cmdline);
static void testmodeHandleWB(uint8_t * cmdline);
static void testmodeHandleDCDC(uint8_t * cmdline);

static void testmode_radio_init(void);
void testmode_radio_disable(void);
void testmode_radio_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel, uint8_t dcdc);
void testmode_radio_modulated_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel, uint8_t dcdc);
void testmode_radio_rx_carrier(uint8_t mode, uint8_t channel);


#define STO_TESTMODE_NUMBER_CMDS 18


const TestModeCmdDefinition testmodeCmdTable[STO_TESTMODE_NUMBER_CMDS] =
{
	{ "HELP",
    testmodeHandleHELP
  },
	{ "BOAD",
    testmodeHandleBOAD
  },
	{ "DEVAD",
    testmodeHandleDEVAD
  },
	{ "VER",
    testmodeHandleVER
  },
	{ "VERB",
    testmodeHandleVERB
  },
	{ "WD",
    testmodeHandleWD
  },
	{ "WW",
    testmodeHandleWW
  },
	{ "WB",
    testmodeHandleWB
  },
	{ "DD",
    testmodeHandleDD
  },
	{ "DW",
    testmodeHandleDW
  },
	{ "DB",
    testmodeHandleDB
  },
	{ "RXON",
    testmodeHandleRXON
  },
	{ "TXCW",
    testmodeHandleTXCW
  },
	{ "TXMOD",
    testmodeHandleTXMOD
  },
	{ "TXPWR",
    testmodeHandleTXPWR
  },
	{ "RFCHAN",
    testmodeHandleRFCHAN
  },
	{ "RFOFF",
    testmodeHandleRFOFF
  },
	{ "DCDC",
    testmodeHandleDCDC
  }
 };

static void testmodeHandleRead(uint8_t * cmdline, uint8_t width)
{
	uint32_t	dword, address;
	uint16_t	word;
	uint8_t	byte;
	char * pEnd;
  
	if(strlen((char*)cmdline) < 2)
	{
		return;
	}
 
	address = strtoul((char*)cmdline,&pEnd,16);
	
	switch(width)
	{
		case 1:
			byte = *((uint8_t*)address);
		  sprintf(pEnd, ": %2.2X",byte);
			break;
		case 2:
      /* make sure address is WORD aligned */
      address &= ~ (0x01UL);
			word = *((uint16_t*)address);
		  sprintf(pEnd, ": %4.4X",word);
			break;
		case 4:
      /* make sure address is DWORD aligned */
      address &= ~ (0x03UL);
			dword = *((uint32_t*)address);
		  sprintf(pEnd, ": %8.8X",dword);
			break;
	}
	testmode_uart_write((uint8_t*)"\r\n", 2);
	testmode_uart_write(cmdline, strlen((char*)cmdline));
}

static void testmodeHandleWrite(uint8_t * cmdline, uint8_t width)
{
	uint32_t	data, address;
	char * pEnd;

	if(strlen((char*)cmdline) < 2)
	{
		return;
	}
  
	address = strtoul((char*)cmdline,&pEnd,16);
	data    = strtoul(pEnd,&pEnd,16);
	
	switch(width)
	{
		case 1:
			*((uint8_t*)address) = (uint8_t)data;
			break;
		case 2:
			*((uint16_t*)address) = (uint16_t)data;
			break;
		case 4:
			*((uint32_t*)address) = (uint32_t)data;
			break;
	}
}

static void testmodeHandleDD(uint8_t * cmdline)
{
	testmodeHandleRead(cmdline, 4);
}
static void testmodeHandleDW(uint8_t * cmdline)
{
	testmodeHandleRead(cmdline, 2);
}
static void testmodeHandleDB(uint8_t * cmdline)
{
	testmodeHandleRead(cmdline, 1);
}
static void testmodeHandleWD(uint8_t * cmdline)
{
	testmodeHandleWrite(cmdline, 4);
}
static void testmodeHandleWW(uint8_t * cmdline)
{
	testmodeHandleWrite(cmdline, 2);
}
static void testmodeHandleWB(uint8_t * cmdline)
{
	testmodeHandleWrite(cmdline, 1);
}

static void testmodeHandleRXON(uint8_t * cmdline)
{
	testmode_radio_rx_carrier(radio_mode, radio_channel);
}

static void testmodeHandleRFOFF(uint8_t * cmdline)
{
	testmode_radio_disable();
}

static void testmodeHandleTXCW(uint8_t * cmdline)
{
	testmode_radio_tx_carrier(radio_txpower, radio_mode, radio_channel, radio_dcdc);
}

static void testmodeHandleTXMOD(uint8_t * cmdline)
{
	testmode_radio_modulated_tx_carrier(radio_txpower, radio_mode, radio_channel, radio_dcdc);
}

static void testmodeHandleTXPWR(uint8_t * cmdline)
{
	switch (cmdline[1])
	{
		case '0':
				radio_txpower =  RADIO_TXPOWER_TXPOWER_Pos4dBm;
				break;
		case '1':
				radio_txpower =  RADIO_TXPOWER_TXPOWER_0dBm;
				break;
		case '2':
				radio_txpower = RADIO_TXPOWER_TXPOWER_Neg4dBm;
				break;
		case '3':
				radio_txpower = RADIO_TXPOWER_TXPOWER_Neg8dBm;
				break;
		case '4':
				radio_txpower = RADIO_TXPOWER_TXPOWER_Neg12dBm;
				break;
		case '5':
				radio_txpower = RADIO_TXPOWER_TXPOWER_Neg16dBm;
				break;
		case '6':
				radio_txpower = RADIO_TXPOWER_TXPOWER_Neg20dBm;
				break;
		case '7':
				// fall through 
		default:
				radio_txpower = RADIO_TXPOWER_TXPOWER_Neg30dBm;
				break;
	}
}

static void testmodeHandleRFCHAN(uint8_t * cmdline)
{
	if( strlen((char*)cmdline) < 4 && strlen((char*)cmdline) >= 2)
	{
		uint8_t channel = 0;
		char * pEnd;

		channel = strtoul((char*)cmdline,&pEnd,10);
		if( channel <= 80)
		{
			radio_channel = channel;
		}
	}
}

static void testmodeHandleDCDC(uint8_t * cmdline)
{
#if defined (NRF51)
	char string[80] = "";

	if( ! ((uint16_t)NRF_FICR->CONFIGID > STO_LAST_NRF51_SECOND_EDITION_HW_ID) )
	{
    sprintf(string, "ERR HW_ID: %4.4X",(uint16_t)NRF_FICR->CONFIGID);
  	testmode_uart_write((uint8_t*)"\r\n", 2);
	  testmode_uart_write((uint8_t*)string, os_strlen(string));
	} else
#endif //defined (NRF51)
 	{
		switch (cmdline[1])
		{
			case '1':
					radio_dcdc =  POWER_DCDCEN_DCDCEN_Enabled;
					testmode_uart_write((uint8_t*)"\r\n", 2);
        	testmode_uart_write((uint8_t*)"dcdc on", 7);
					break;
			case '0':
			default:
					radio_dcdc = POWER_DCDCEN_DCDCEN_Disabled;
        	testmode_uart_write((uint8_t*)"\r\n", 2);
        	testmode_uart_write((uint8_t*)"dcdc off", 8);
					break;
		}
	}
}


static void testmodeHandleVER(uint8_t * cmdline)
{
	/* ToDo : insert your firmware version string for the production test here*/
	
	 uint8_t string[80] = "\r\n Please replace with your version string";
	 testmode_uart_write(string, strlen((char *)string));
}

static void testmodeHandleVERB(uint8_t * cmdline)
{
	/* ToDo : insert your bootloader version string for the production test here*/
	
	 uint8_t string[80] = "\r\n Please replace with your version string";
	 testmode_uart_write(string, strlen((char *)string));
}

static void testmodeHandleBOAD(uint8_t * cmdline)
{
	  uint8_t string[0x80] = "\r\n";
    char    hex[] = "0123456789ABCDEF";
		uint8_t	temp = 0;
    uint8_t addr[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
		uint8_t *p_UICR = (uint8_t *)(NRF_UICR);

	
	if(strlen((char*)cmdline) == 13 && cmdline[0] == '=')
		{
  		p_UICR += 0x80;  /*UICR_CUSTOMER_RESERVED_OFFSET;*/
			
			btStringToBd((char *)&cmdline[1], addr);
			
 		  ble_flash_block_write((uint32_t *) p_UICR, (uint32_t *) addr, 2);
			
		} else
		if(strlen((char*)cmdline) == 0)
		{
  		p_UICR += 0x80;

			if(p_UICR[0] == 0xFF && p_UICR[1] == 0xFF && p_UICR[2] == 0xFF &&
				 p_UICR[3] == 0xFF && p_UICR[4] == 0xFF && p_UICR[5] == 0xFF)			
			{
				temp = (NRF_FICR->DEVICEADDR[1] & 0x0000FF00) >> 8;
				string[2] = hex[(temp >> 4)|0xc] ;
				string[3] = hex[temp & 0x0F];
				temp = (NRF_FICR->DEVICEADDR[1] & 0x000000FF);
				string[4] = hex[temp >> 4];
				string[5] = hex[temp & 0x0F];
				temp = (NRF_FICR->DEVICEADDR[0] & 0xFF000000) >> 24;
				string[6] = hex[temp >> 4];
				string[7] = hex[temp & 0x0F];
				temp = (NRF_FICR->DEVICEADDR[0] & 0x00FF0000) >> 16;
				string[8] = hex[temp >> 4];
				string[9] = hex[temp & 0x0F];
				temp = (NRF_FICR->DEVICEADDR[0] & 0x0000FF00) >> 8;
				string[10] = hex[temp >> 4];
				string[11] = hex[temp & 0x0F];
				temp = (NRF_FICR->DEVICEADDR[0] & 0x000000FF);
				string[12] = hex[temp >> 4];
				string[13] = hex[temp & 0x0F];
				string[14] = 0;
			} else
			{	
				temp = p_UICR[0];
				string[2] = hex[(temp >> 4)] ;
				string[3] = hex[temp & 0x0F];
				temp = p_UICR[1];
				string[4] = hex[temp >> 4];
				string[5] = hex[temp & 0x0F];
				temp = p_UICR[2];
				string[6] = hex[temp >> 4];
				string[7] = hex[temp & 0x0F];
				temp = p_UICR[3];
				string[8] = hex[temp >> 4];
				string[9] = hex[temp & 0x0F];
				temp = p_UICR[4];
				string[10] = hex[temp >> 4];
				string[11] = hex[temp & 0x0F];
				temp = p_UICR[5];
				string[12] = hex[temp >> 4];
				string[13] = hex[temp & 0x0F];
				string[14] = 0;
			}
			
			testmode_uart_write(string, strlen((char *)string));
			
		}
		else
		{
  	  testmode_uart_write((uint8_t*)"\r\nERROR", 7);	
			return;
		}

		
}
static void testmodeHandleDEVAD(uint8_t * cmdline)
{
	  uint8_t string[0x80] = "\r\n";
    char    hex[] = "0123456789ABCDEF";
		uint8_t	temp = 0;
	
		if(strlen((char*)cmdline) == 0)
		{
			temp = (NRF_FICR->DEVICEADDR[1] & 0x0000FF00) >> 8;
			string[2] = hex[(temp >> 4)|0xc] ;
			string[3] = hex[temp & 0x0F];
			temp = (NRF_FICR->DEVICEADDR[1] & 0x000000FF);
			string[4] = hex[temp >> 4];
			string[5] = hex[temp & 0x0F];
			temp = (NRF_FICR->DEVICEADDR[0] & 0xFF000000) >> 24;
			string[6] = hex[temp >> 4];
			string[7] = hex[temp & 0x0F];
			temp = (NRF_FICR->DEVICEADDR[0] & 0x00FF0000) >> 16;
			string[8] = hex[temp >> 4];
			string[9] = hex[temp & 0x0F];
			temp = (NRF_FICR->DEVICEADDR[0] & 0x0000FF00) >> 8;
			string[10] = hex[temp >> 4];
			string[11] = hex[temp & 0x0F];
			temp = (NRF_FICR->DEVICEADDR[0] & 0x000000FF);
			string[12] = hex[temp >> 4];
			string[13] = hex[temp & 0x0F];
			string[14] = 0;
			
			testmode_uart_write(string, strlen((char *)string));
	
		}
		else
		{
  	  testmode_uart_write((uint8_t*)"\r\nERROR", 7);	
			return;
		}
}
		
static void testmodeHandleHELP(uint8_t * cmdline)
{
		int i;
	  uint8_t string[80] = "\r\n";
		for(i=0; i < STO_TESTMODE_NUMBER_CMDS; i++)
		{
			strcpy((char*)&string[2], testmodeCmdTable[i].cmd);
			testmode_uart_write( string, strlen((char *)string));	
		}
}

static bool testmodeiCmp(uint8_t * inputLine, char * pCmd)
{
  bool result = true;
  int i,j;
  j = strlen(pCmd);
  for(i = 0;i < j; i++)
  {
    if(toupper(inputLine[i]) != pCmd[i])
    {
      result = false;
      break;
    }
  }
  return result;
}

static void testmodeParseCommand(uint8_t * inputLine )
 {
  int   i = 0;

  if (strlen((char *)inputLine) != 0 /*&& stoDevCheckBDaddress()*/)
   {
    /* now search command */

    for (i = STO_TESTMODE_NUMBER_CMDS-1 ; i >= 0 && inputLine != NULL; i--)
     {
      if (testmodeiCmp(inputLine, (char*)testmodeCmdTable[i].cmd) )
        {
        /* call handler has to free inputLine !!! */
        testmodeCmdTable[i].cmdHandler(&inputLine[strlen((char*)testmodeCmdTable[i].cmd)]);
				break;
       }
     }

    if (i < 0)
     {
			testmode_uart_write( (uint8_t*)PARSER_UKNOWN_CMD, strlen(PARSER_UKNOWN_CMD));
     }
   }
	 testmode_uart_write( (uint8_t*)PARSER_PROMPT, PARSER_PROMPT_LEN);
 }

 
static uint32_t testmode_uart_write( const uint8_t * p_buffer, uint32_t length)
{
	int i;
	
	for(i=0; i < length; i++)
	{
		if( app_uart_put(p_buffer[i]) != NRF_SUCCESS)
		{ 
			break;
		}
	}
	return i;
}
 
 
static void testmode_uart_event_handler(app_uart_evt_t * uart_event)
{
    switch (uart_event->evt_type)
    {
			case APP_UART_DATA_READY:   
				{
					uint16_t  rxReadBytes;

					for(rxReadBytes = 0; rxReadBytes < TESTMODE_CMDLINE_SIZE-inputCmdLinePos; rxReadBytes++)
					{					
						if( app_uart_get(&inputCmdLine[inputCmdLinePos+rxReadBytes]) != NRF_SUCCESS)
						{
							break;
						}
					}
					
					int i;
					/* echo */
					for(i= 0; i<rxReadBytes;i++)
					{
						testmode_uart_write(&inputCmdLine[inputCmdLinePos], 1);
  					/* process */
						if(inputCmdLine[inputCmdLinePos] == 0x0d)						
						{
							inputCmdLine[inputCmdLinePos] = 0;
							testmodeParseCommand(inputCmdLine);
						  inputCmdLinePos = 0;
						} else if(inputCmdLine[inputCmdLinePos] == 0x08)						
						{
							uint8_t data[2] = { 0x20, 0x08 };				
  						testmode_uart_write(data, 2);
							inputCmdLinePos--;
						} else
						{
						  inputCmdLinePos++;
						}
					}				
				}
				break;
      case APP_UART_TX_EMPTY:                       /**< An event indicating write completion of the TX packet provided in the function call \ref testmode_uart_write . */
				break;
      case APP_UART_COMMUNICATION_ERROR:            /**< An event indicating that an unrecoverable error has occurred. */
				break;
			default:
				break;
		}
}

static void start_firmware( void )
{
  if(m_current_state != TESTMODE_OFF)
  {
    NVIC_SystemReset(); /* leave testmode via reset */
  }
}

static void start_sto_testmode( void )
{
  if(m_current_state == TESTMODE_OFF)
  {
    m_current_state = TESTMODE_ON;
    radio_packet   = malloc(0x100);
    inputCmdLine   = malloc(TESTMODE_CMDLINE_SIZE);
    m_uartRxBuffer = malloc(UART_MaxBufferSize);
    m_uartTxBuffer = malloc(UART_MaxBufferSize);
    testmode_radio_init();
    app_uart_comm_params_t comm_params =
    {
      RX_PIN_NUMBER,
      TX_PIN_NUMBER,
      RTS_PIN_NUMBER,
      CTS_PIN_NUMBER,
#if (HWFC == true)
      APP_UART_FLOW_CONTROL_ENABLED,
#else								
      APP_UART_FLOW_CONTROL_DISABLED,
#endif								
      false,
      UART_BAUDRATE_BAUDRATE_Baud38400
    };
							
    app_uart_buffers_t buffers = { m_uartRxBuffer, UART_MaxBufferSize, m_uartTxBuffer, UART_MaxBufferSize };
    app_uart_init(&comm_params,
                  &buffers,
                  testmode_uart_event_handler,
                  APP_IRQ_PRIORITY_HIGH);

    while(true)
    {
      ;
    }
  } else
  {
    NVIC_SystemReset(); /* leave testmode to defined bootstate via reset */
  }
}

static void start_direct_testmode( void )
{
  if(m_current_state == TESTMODE_OFF)
  {
    m_current_state = DTMMODE_ON;
    dtmmode_main();
  }					
}

static void start_bootloader( void )
{
  if(m_current_state == TESTMODE_OFF)
  {
    m_current_state = FLASHMODE_ON;
    NVIC_SystemReset(); /* not allowed !! */
  } else
  {
    NVIC_SystemReset(); /* leave testmode to defined bootstate via reset */
  }				
}

static void testmode_gpio_event_handler(nrf_drv_gpiote_pin_t pin,
                                        nrf_gpiote_polarity_t action)
{
  uint32_t active_pins = 0;

  if ( m_pin_io_mode_mask & (1 << pin) )
  {
    if ( nrf_drv_gpiote_in_is_set(TESTMODE) )
      active_pins |= (1 << TESTMODE);
    if ( nrf_drv_gpiote_in_is_set(BOOT0) )
      active_pins |= (1 << BOOT0);

    switch (active_pins)
    {
      case START_FIRMWARE:
        start_firmware();
        break;
      case START_STO_TESTMODE:
        start_sto_testmode();
        break;
      case START_DIRECT_TESTMODE:
        start_direct_testmode();
        break;
      case START_BOOT_LOADER:
        start_bootloader();
        break;
      default:
        // All valid events are handled above.
        break;
    }
  }
}


uint32_t testmode_init( void)
{
		uint32_t err_code;
		uint32_t gpiote_pin_low_high_mask = 0;
		uint32_t gpiote_pin_high_low_mask = 0;
		
	  m_current_state = TESTMODE_OFF;

    inputCmdLinePos = 0;
	
		m_pin_io_mode_mask = (1 << TESTMODE);
		m_pin_io_mode_mask |= (1 << BOOT0);
	
    gpiote_pin_low_high_mask  = (1 << TESTMODE);
		gpiote_pin_low_high_mask |= (1 << BOOT0);
		
		gpiote_pin_high_low_mask  = (1 << TESTMODE);
		gpiote_pin_high_low_mask |= (1 << BOOT0);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    {
        int i;

        /* configure pins */      
        for (i=0; i<(sizeof(testmode_pin_cfg) / sizeof(testmode_pin_cfg_t)); i++)
        {
            err_code = nrf_drv_gpiote_in_init(
                            testmode_pin_cfg[i].pin_no,
                            &testmode_pin_cfg[i].pin_cfg,
                            testmode_gpio_event_handler);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }
        
        /* enable sense events for pins */      
        for (i=0; i<(sizeof(testmode_pin_cfg) / sizeof(testmode_pin_cfg_t)); i++)
        {
            nrf_drv_gpiote_in_event_enable(testmode_pin_cfg[i].pin_no, true);
        }
    }

		return NRF_SUCCESS;
}


uint32_t testmode_check(void)
{
  /* fake pin sense event */
  testmode_gpio_event_handler(TESTMODE, NRF_GPIOTE_POLARITY_HITOLO);
  m_current_state = TESTMODE_IDLE;
	
	return NRF_SUCCESS;
}


uint32_t testmode_close(void)
{
  int      i;
  
  nrf_gpio_cfg_disable(TESTMODE);
  nrf_gpio_cfg_disable(BOOT0);
	
  for (i=0; i<(sizeof(testmode_pin_cfg) / sizeof(testmode_pin_cfg_t)); i++)
  {
    nrf_drv_gpiote_in_event_disable(testmode_pin_cfg[i].pin_no);
  }
  m_current_state = TESTMODE_OFF;

	if(radio_packet != NULL) free(radio_packet);
	if(inputCmdLine != NULL) free(inputCmdLine);
	if(m_uartRxBuffer != NULL) free(m_uartRxBuffer);
	if(m_uartTxBuffer != NULL) free(m_uartTxBuffer);
	m_uartRxBuffer = NULL;
	m_uartTxBuffer = NULL;
	inputCmdLine = NULL;
	radio_packet = NULL;
	
	return NRF_SUCCESS;
}


static void testmode_radio_init(void)
{
    NRF_RNG->TASKS_START            = 1;
    
    // Start 16 MHz crystal oscillator
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }  
}

void testmode_radio_disable(void)
{
    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
#if defined (NRF51)  
    NRF_RADIO->TEST            = 0;
#elif defined(NRF52)
    NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos) |
                          (RADIO_MODECNF0_DTX_B1 << RADIO_MODECNF0_DTX_Pos);
#endif // NRF51 / NRF52
    NRF_RADIO->TASKS_DISABLE   = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0)
    {
        // Do nothing.
    }
    NRF_RADIO->EVENTS_DISABLED = 0;
}

void testmode_radio_rx_carrier(uint8_t mode, uint8_t channel)
{
    testmode_radio_disable();
    NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->FREQUENCY  = channel;
    NRF_RADIO->TASKS_RXEN = 1;
}


void testmode_radio_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel, uint8_t dcdc)
{
    testmode_radio_disable();
#if defined (NRF51)
		if( ! ((uint16_t)NRF_FICR->CONFIGID > STO_LAST_NRF51_SECOND_EDITION_HW_ID) )
		{
			NRF_POWER->DCDCEN			= POWER_DCDCEN_DCDCEN_Disabled;	
		} else
#endif //defined (NRF51)  
  	{
			NRF_POWER->DCDCEN			= (dcdc << POWER_DCDCEN_DCDCEN_Pos);
		}
    NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->TXPOWER    = (txpower << RADIO_TXPOWER_TXPOWER_Pos);    
    NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;
#if defined (NRF51)    
    NRF_RADIO->TEST       = (RADIO_TEST_CONST_CARRIER_Enabled << RADIO_TEST_CONST_CARRIER_Pos) \
                          | (RADIO_TEST_PLL_LOCK_Enabled << RADIO_TEST_PLL_LOCK_Pos);
#elif defined(NRF52)
    NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos) |
                          (RADIO_MODECNF0_DTX_B1 << RADIO_MODECNF0_DTX_Pos);
#endif // NRF51 / NRF52
    NRF_RADIO->TASKS_TXEN = 1;
}

static uint32_t rnd8(void)
{
    NRF_RNG->EVENTS_VALRDY = 0;
    while(NRF_RNG->EVENTS_VALRDY == 0)
    {
        // Do nothing.
    }
    return  NRF_RNG->VALUE;
}

static uint32_t rnd32(void)
{
    uint8_t i;
    uint32_t val = 0;

    for(i=0; i<4; i++)
    {
        val <<= 8;
        val |= rnd8();
    }
    return val;
}

static void testmode_generate_modulated_rf_packet(void)
{
    uint8_t i;

    NRF_RADIO->PREFIX0 = rnd8();
    NRF_RADIO->BASE0   = rnd32();

    // Packet configuration:
    // S1 size = 0 bits, S0 size = 0 bytes, payload length size = 8 bits
    NRF_RADIO->PCNF0  = (0UL << RADIO_PCNF0_S1LEN_Pos) |
                       (0UL << RADIO_PCNF0_S0LEN_Pos) |
                       (8UL << RADIO_PCNF0_LFLEN_Pos);
    // Packet configuration:
    // Bit 25: 1 Whitening enabled
    // Bit 24: 1 Big endian,
    // 4 byte base address length (5 byte full address length), 
    // 0 byte static length, max 255 byte payload .
    NRF_RADIO->PCNF1  = (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos) |
                        (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
                        (4UL << RADIO_PCNF1_BALEN_Pos) |
                        (0UL << RADIO_PCNF1_STATLEN_Pos) |
                       (255UL << RADIO_PCNF1_MAXLEN_Pos);
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);
    radio_packet[0]         = 254;    // 254 byte payload.
  
    // Fill payload with random data.
    for(i = 0; i < 254; i++)
    {
        radio_packet[i+1] = rnd8();
    }
    NRF_RADIO->PACKETPTR = (uint32_t)radio_packet;
}

void testmode_radio_modulated_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel, uint8_t dcdc)
{
    testmode_radio_disable();
    testmode_generate_modulated_rf_packet();
#if defined (NRF51)
		if( ! ((uint16_t)NRF_FICR->CONFIGID > STO_LAST_NRF51_SECOND_EDITION_HW_ID) )
		{
			NRF_POWER->DCDCEN			= POWER_DCDCEN_DCDCEN_Disabled;	
		} else
#endif //defined (NRF51)  
  	{
			NRF_POWER->DCDCEN			= (dcdc << POWER_DCDCEN_DCDCEN_Pos);
		}

    NRF_RADIO->SHORTS     = RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_READY_START_Msk | \
                            RADIO_SHORTS_DISABLED_TXEN_Msk;;
    NRF_RADIO->TXPOWER    = (txpower << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;
    NRF_RADIO->TASKS_TXEN = 1;
}
