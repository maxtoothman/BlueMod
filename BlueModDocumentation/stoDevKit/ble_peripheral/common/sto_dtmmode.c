/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
 
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "ble_dtm.h"
#include <boards.h>

void testmode_POFInit(void);

// Configuration parameters.
#define UART_TX_PIN     TX_PIN_NUMBER                     /**< Pin used for UART Transmit. */
#define UART_RX_PIN     RX_PIN_NUMBER                     /**< Pin used for UART Receive. */
#define BITRATE         UART_BAUDRATE_BAUDRATE_Baud19200  /**< Serial bitrate on the UART */

// @note: The BLE DTM 2-wire UART standard specifies 8 data bits, 1 stop bit, no flow control.
//        These parameters are not configurable in the BLE standard.

/**@details Maximum iterations needed in the main loop between stop bit 1st byte and start bit 2nd
 * byte. DTM standard allows 5000us delay between stop bit 1st byte and start bit 2nd byte. 
 * As the time is only known when a byte is received, then the time between between stop bit 1st 
 * byte and stop bit 2nd byte becomes: 
 *      5000us + transmission time of 2nd byte.
 *
 * Byte transmission time is (Baud rate of 19200):
 *      10bits * 1/19200 = approx. 520 us/byte (8 data bits + start & stop bit).
 *
 * Loop time on polling UART register for received byte is defined in ble_dtm.c as:
 *   UART_POLL_CYCLE = 260 us
 *
 * The max time between two bytes thus becomes (loop time: 260us / iteration): 
 *      (5000us + 520us) / 260us / iteration = 21.2 iterations. 
 *
 * This is rounded down to 21. 
 *
 * @note If UART bit rate is changed, this value should be recalculated as well.
 */
#define MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE 21

/**@brief Function for configuration of pin to use for UART TX.
 *
 * @param[in] pin_number    The pin to use as TX.
 */
static __INLINE void dtmmode_nrf_gpio_cfg_output(uint32_t pin_number)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
                                    (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                    (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                    (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
}


/**@brief Function for configuration of pin to use for UART RX.
 *
 * @param[in] pin_number    The pin to use as RX.
 * @param[in] pull_config   Configuration of pull resistor.
 */
static __INLINE void dtmmode_nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | 
                                    (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                    (pull_config << GPIO_PIN_CNF_PULL_Pos) |
                                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                    (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}


/**@brief Function for UART initialization.
 */
static void dtmmode_uart_init(void)
{
    // Configure UART0 pins.
    dtmmode_nrf_gpio_cfg_output(UART_TX_PIN);
    dtmmode_nrf_gpio_cfg_input(UART_RX_PIN, NRF_GPIO_PIN_NOPULL);  

    NRF_UART0->PSELTXD         = UART_TX_PIN;
    NRF_UART0->PSELRXD         = UART_RX_PIN;
    NRF_UART0->BAUDRATE        = BITRATE;

    // Clean out possible events from earlier operations
    NRF_UART0->EVENTS_RXDRDY   = 0;
    NRF_UART0->EVENTS_TXDRDY   = 0;
    NRF_UART0->EVENTS_ERROR    = 0;

    // Activate UART.
    NRF_UART0->ENABLE          = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->INTENSET        = 0;
    NRF_UART0->TASKS_STARTTX   = 1;
    NRF_UART0->TASKS_STARTRX   = 1;
}


/**@brief Function for splitting UART command bit fields into separate command parameters for the DTM library.
*
 * @param[in]   command   The packed UART command.
 * @return      result status from dtmlib.
 */
static uint32_t dtmmode_dtm_cmd_put(uint16_t command)
{
    dtm_cmd_t      command_code = (command >> 14) & 0x03;
    dtm_freq_t     freq         = (command >> 8) & 0x3F;
    uint32_t       length       = (command >> 2) & 0x3F;
    dtm_pkt_type_t payload      = command & 0x03;
  
    // Check for Vendor Specific payload.
    if (payload == 0x03) 
    {
        /* Note that in a HCI adaption layer, as well as in the DTM PDU format,
           the value 0x03 is a distinct bit pattern (PRBS15). Even though BLE does not
           support PRBS15, this implementation re-maps 0x03 to DTM_PKT_VENDORSPECIFIC,
           to avoid the risk of confusion, should the code be extended to greater coverage. 
        */
        payload = DTM_PKT_VENDORSPECIFIC;
    }
    return dtm_cmd(command_code, freq, length, payload);
}


/**@brief Function for application main entry.
 *
 * @details This function serves as an adaptation layer between a 2-wire UART interface and the
 *          dtmlib. After initialization, DTM commands submitted through the UART are forwarded to
 *          dtmlib and events (i.e. results from the command) is reported back through the UART.
 */
int dtmmode_main(void)
{
    uint32_t    current_time;
    uint32_t    dtm_error_code;
    uint32_t    msb_time = 0;          // Time when MSB of the DTM command was read. Used to catch stray bytes from "misbehaving" testers.
    bool        is_msb_read = false;   // True when MSB of the DTM command has been read and the application is waiting for LSB.
    uint16_t    dtm_cmd_from_uart = 0; // Packed command containing command_code:freqency:length:payload in 2:6:6:2 bits.
    uint8_t     rx_byte;               // Last byte read from UART.
    dtm_event_t result;                // Result of a DTM operation.

    dtmmode_uart_init();

    dtm_error_code = dtm_init();
    if (dtm_error_code != DTM_SUCCESS)
    {
        // If DTM cannot be correctly initialized, then we just return.
        return -1;
    }

		testmode_POFInit();

    for (;;)
    {
        // Will return every timeout, 625 us.
        current_time = dtm_wait();  

        if (NRF_UART0->EVENTS_RXDRDY == 0)
        {
            // Nothing read from the UART.
            continue;
        }
        NRF_UART0->EVENTS_RXDRDY = 0;
        rx_byte                  = (uint8_t)NRF_UART0->RXD;

        if (!is_msb_read)
        {
            // This is first byte of two-byte command.
            is_msb_read       = true;
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;

            // Go back and wait for 2nd byte of command word.
            continue;
        }

        // This is the second byte read; combine it with the first and process command
        if (current_time > (msb_time + MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE))
        {
            // More than ~5mS after msb: Drop old byte, take the new byte as MSB.
            // The variable is_msb_read will remains true.
            // Go back and wait for 2nd byte of the command word.
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;
            continue;
        }

        // 2-byte UART command received.
        is_msb_read        = false;
        dtm_cmd_from_uart |= (dtm_cmd_t)rx_byte;

        if (dtmmode_dtm_cmd_put(dtm_cmd_from_uart) != DTM_SUCCESS)
        {
            // Extended error handling may be put here. 
            // Default behavior is to return the event on the UART (see below);
            // the event report will reflect any lack of success.
        }

        // Retrieve result of the operation. This implementation will busy-loop
        // for the duration of the byte transmissions on the UART.
        if (dtm_event_get(&result))
        {
            // Report command status on the UART.
            // Transmit MSB of the result.
            NRF_UART0->TXD = (result >> 8) & 0xFF;
            // Wait until MSB is sent.
            while (NRF_UART0->EVENTS_TXDRDY != 1)
            {
                // Do nothing.
            }
            NRF_UART0->EVENTS_TXDRDY = 0;

            // Transmit LSB of the result.
            NRF_UART0->TXD = result & 0xFF;
            // Wait until LSB is sent.
            while (NRF_UART0->EVENTS_TXDRDY != 1)
            {
                // Do nothing.
            }
            NRF_UART0->EVENTS_TXDRDY = 0;
        }
    }
}

/**@brief interrupt handler. */
void POWER_CLOCK_IRQHandler(void)
{
	if(NRF_POWER->EVENTS_POFWARN == 1)
	{
		NRF_POWER->DCDCEN			= POWER_DCDCEN_DCDCEN_Disabled;	
		nrf_gpio_pin_set(IOB);
		NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
	}
}

void testmode_POFInit(void)
{
	NRF_POWER->POFCON = ((POWER_POFCON_THRESHOLD_V23 << POWER_POFCON_THRESHOLD_Pos) | (POWER_POFCON_POF_Enabled << POWER_POFCON_POF_Pos));
	NRF_POWER->INTENSET = (POWER_INTENSET_POFWARN_Enabled << POWER_INTENSET_POFWARN_Pos);
	//NRF_POWER->INTENCLR = (POWER_INTENCLR_POFWARN_Enabled << POWER_INTENCLR_POFWARN_Pos);
	
  NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
	NVIC_EnableIRQ(POWER_CLOCK_IRQn);
	
  nrf_gpio_cfg_output(IOB);
	nrf_gpio_pin_clear(IOB);
}




/// @}
