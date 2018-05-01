/*
 * Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/** 
 * @brief BLE Heart Rate Collector application main file.
 *
 * This file contains the source code for a sample heart rate collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "pstorage.h"
#include "device_manager.h"
#include "our_service_c.h"
#include "ble_bas_c.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_log.h"

#define CENTRAL_LINK_COUNT         1                                  /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT      0                                  /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define STRING_BUFFER_LEN          50
#define BOND_DELETE_ALL_BUTTON_ID  0                                  /**< Button used for deleting all bonded centrals during startup. */

#define APP_TIMER_PRESCALER        0                                  /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE    2                                  /**< Size of timer operation queues. */

#define APPL_LOG                   NRF_LOG_PRINTF                     /**< Logger macro that will be used in this file to do logging over UART or RTT based on nrf_log configuration. */
#define APPL_LOG_DEBUG             NRF_LOG_PRINTF_DEBUG               /**< Debug logger macro that will be used in this file to do logging of debug information over UART or RTT based on nrf_log configuration. This will only work if DEBUG is defined*/

#define SEC_PARAM_BOND             1                                  /**< Perform bonding. */
#define SEC_PARAM_MITM             1                                  /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC             0                                  /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS         0                                  /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES  BLE_GAP_IO_CAPS_NONE               /**< No I/O capabilities. */
#define SEC_PARAM_OOB              0                                  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE     7                                  /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE     16                                 /**< Maximum encryption key size. */

#define SCAN_INTERVAL              0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL    MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL    MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY              0                                  /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 millisecond. */

#define TARGET_UUID                0xF00D             /**< Target device name that application is looking for. */
#define UUID16_SIZE                2                                  /**< Size of 16 bit UUID */
#define UUID32_SIZE								 4
#define UUID128_SIZE							 16


#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */

/*static const ble_gap_scan_params_t m_scan_params = 
  {
    .active      = SCAN_ACTIVE,
    .selective   = SCAN_SELECTIVE,
    .p_whitelist = NULL,
    .interval    = SCAN_INTERVAL,
    .window      = SCAN_WINDOW,
    .timeout     = SCAN_TIMEOUT
  };
	*/

static const ble_uuid_t m_our_service_uuid = 
  {
    .uuid = 0xF00D,
    .type = BLE_UUID_TYPE_VENDOR_BEGIN  
  };

/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t     * p_data;                                             /**< Pointer to data. */
    uint16_t      data_len;                                           /**< Length of data. */
}data_t;

typedef enum
{
    BLE_NO_SCAN,                                                     /**< No advertising running. */
    BLE_WHITELIST_SCAN,                                              /**< Advertising with whitelist. */
    BLE_FAST_SCAN,                                                   /**< Fast advertising running. */
} ble_scan_mode_t;

static ble_db_discovery_t           m_ble_db_discovery;                  /**< Structure used to identify the DB Discovery module. */

//ble_our_service_c_t is the same as ble_our_service_c_s
//Defines Client structure, includes uuid, conn_handle, peer_our_service_db, evt_handler
static ble_our_service_c_t          m_ble_our_service_c;                 /**< Structure used to identify the sensor client module. */

//static ble_bas_c_t                  m_ble_bas_c;                       /**< Structure used to identify the Battery Service client module. */
static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
static dm_application_instance_t    m_dm_app_id;                         /**< Application identifier. */
static dm_handle_t                  m_dm_device_handle;                  /**< Device Identifier identifier. */
static uint8_t                      m_peer_count = 0;                    /**< Number of peer's connected. */
static ble_scan_mode_t              m_scan_mode = BLE_FAST_SCAN;         /**< Scan mode used by application. */
static uint16_t                     m_conn_handle;                       /**< Current connection handle. */
static volatile bool                m_whitelist_temporarily_disabled = false; /**< True if whitelist has been temporarily disabled. */

static bool                         m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */

static bool is_uuid_present(const ble_uuid_t *p_target_uuid, 
                            const ble_gap_evt_adv_report_t *p_adv_report);

/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    0,                                   // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

static void scan_start(void);


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
		printf("db_disc_handler running");
    ble_our_service_c_on_db_disc_evt(&m_ble_our_service_c, p_evt);
}


/**@brief Callback handling device manager events.
 *
 * @details This function is called to notify the application of device manager events.
 *
 * @param[in]   p_handle      Device Manager Handle. For link related events, this parameter
 *                            identifies the peer.
 * @param[in]   p_event       Pointer to the device manager event.
 * @param[in]   event_status  Status of the event.
 */
static ret_code_t device_manager_event_handler(const dm_handle_t    * p_handle,
                                                 const dm_event_t     * p_event,
                                                 const ret_code_t     event_result)
{
    uint32_t err_code;

    switch (p_event->event_id)
    {
        case DM_EVT_CONNECTION:
        {
            APPL_LOG("[APPL]: >> DM_EVT_CONNECTION\r\n");
#ifdef ENABLE_DEBUG_LOG_SUPPORT
            ble_gap_addr_t * peer_addr;
            peer_addr = &p_event->event_param.p_gap_param->params.connected.peer_addr;
            APPL_LOG("[APPL]:[%02X %02X %02X %02X %02X %02X]: Connection Established\r\n",
                                peer_addr->addr[0], peer_addr->addr[1], peer_addr->addr[2],
                                peer_addr->addr[3], peer_addr->addr[4], peer_addr->addr[5]);
#endif // ENABLE_DEBUG_LOG_SUPPORT
            
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_event->event_param.p_gap_param->conn_handle;

            m_dm_device_handle = (*p_handle);

            // Initiate bonding.
            err_code = dm_security_setup_req(&m_dm_device_handle);
            APP_ERROR_CHECK(err_code);

            m_peer_count++;

            if (m_peer_count < CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_CONNECTION\r\n");
            break;
        }

        case DM_EVT_DISCONNECTION:
        {
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_DISCONNECTION\r\n");
            memset(&m_ble_db_discovery, 0 , sizeof (m_ble_db_discovery));

            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            if (m_peer_count == CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            m_peer_count--;
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_DISCONNECTION\r\n");
            break;
        }

        case DM_EVT_SECURITY_SETUP:
        {
            APPL_LOG_DEBUG("[APPL]:[0x%02X] >> DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            // Slave securtiy request received from peer, if from a non bonded device, 
            // initiate security setup, else, wait for encryption to complete.
            err_code = dm_security_setup_req(&m_dm_device_handle);
            APP_ERROR_CHECK(err_code);
            APPL_LOG_DEBUG("[APPL]:[0x%02X] << DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            break;
        }

        case DM_EVT_SECURITY_SETUP_COMPLETE:
        {
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_SECURITY_SETUP_COMPLETE\r\n");
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_SECURITY_SETUP_COMPLETE\r\n");
            break;
        }

        case DM_EVT_LINK_SECURED:
            APPL_LOG_DEBUG("[APPL]: >> DM_LINK_SECURED_IND\r\n");
            // Discover peer's services. 
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                                              p_event->event_param.p_gap_param->conn_handle);
            APP_ERROR_CHECK(err_code);
            APPL_LOG_DEBUG("[APPL]: << DM_LINK_SECURED_IND\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_LOADED:
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_LINK_SECURED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_DEVICE_CONTEXT_LOADED\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_STORED:
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_DEVICE_CONTEXT_STORED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_DEVICE_CONTEXT_STORED\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_DELETED:
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_DEVICE_CONTEXT_DELETED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_DEVICE_CONTEXT_DELETED\r\n");
            break;

        default:
            break;
    }

    return NRF_SUCCESS;
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
/*
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}
*/

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
		APPL_LOG("On BLE Event\r\n");
    uint32_t                err_code;
    const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
				{
						//APPL_LOG("BLE GAP Evt Adv Report\r\n");
						const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
						if (p_adv_report->peer_addr.addr[0] == 0x8E) {
						//APPL_LOG("Address is ");
						for (uint8_t addrindex = 0; addrindex<6; addrindex++) {
							//APPL_LOG("%x",(p_adv_report->peer_addr.addr)[addrindex]);
						}
						//APPL_LOG("\r\n");
						//APPL_LOG("Data Length is %x\r\n", (p_adv_report->dlen));
						//APPL_LOG("Data is ");
						for(uint8_t index = 0; index<p_adv_report->dlen; index++) {
							//APPL_LOG("%x\r\n",(p_adv_report->data)[index]);
						}
						//APPL_LOG("\r\n");
					  }
						//APPL_LOG("BLE Event - BLE GAP EVT ADV Report\r\n");
            if (is_uuid_present(&m_our_service_uuid, p_adv_report))
						{
								// Stop scanning.
								//err_code = sd_ble_gap_scan_stop();

								//if (err_code != NRF_SUCCESS)
								//{
								//		APPL_LOG("[APPL]: Scan stop failed, reason %d\r\n", err_code);
								//}
								
								//err_code = bsp_indication_set(BSP_INDICATE_IDLE);
								//APP_ERROR_CHECK(err_code);

								//m_scan_param.selective = 0; 
								//m_scan_param.p_whitelist = NULL;

								// Initiate connection.
								err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
																							&m_scan_param,
																							&m_connection_param);

								//m_whitelist_temporarily_disabled = false;

								if (err_code != NRF_SUCCESS)
								{
										APPL_LOG("[APPL]: Connection Request Failed, reason %d\r\n", err_code);
								} else {
									APPL_LOG("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
										 p_adv_report->peer_addr.addr[0],
										 p_adv_report->peer_addr.addr[1],
										 p_adv_report->peer_addr.addr[2],
										 p_adv_report->peer_addr.addr[3],
										 p_adv_report->peer_addr.addr[4],
										 p_adv_report->peer_addr.addr[5]
										 );
								}
								break;
						}
				}
					
        case BLE_GAP_EVT_TIMEOUT:
				{
						APPL_LOG("BLE Event - BLE GAP EVT Timeout\r\n");
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                APPL_LOG("[APPL]: Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                APPL_LOG("[APPL]: Connection Request timed out.\r\n");
            }
            break;
					}
        case BLE_GAP_EVT_CONNECTED:
        {
						APPL_LOG("Connected to Target\r\n");
            err_code = ble_our_service_c_handles_assign(&m_ble_our_service_c, p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
						err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            APPL_LOG("About to break\r\n");
						break;
        }
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
				{
            // Accepting parameters requested by peer.
						APPL_LOG("BLE GAP Connection Parameter Update Request\r\n");
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;
				}
        default:
            break;
    }
}

static bool is_uuid_present(const ble_uuid_t *p_target_uuid, 
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint32_t err_code;
    uint32_t index = 0;
    uint8_t *p_data = (uint8_t *)p_adv_report->data;
    ble_uuid_t extracted_uuid;
    
    while (index < p_adv_report->dlen)
    {
				uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];
				if (p_adv_report->peer_addr.addr[0] == 0x8E) {
        //printf("Field Type is %x\r\n",field_type);
				//printf("Field Length is %x\r\n",field_length);
				}
        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE)
           )
        {
						//APPL_LOG("Decoding 16\r\n");
            for (uint32_t u_index = 0; u_index < (field_length/UUID16_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(  UUID16_SIZE, 
                                                &p_data[u_index * UUID16_SIZE + index + 2], 
                                                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
										//APPL_LOG("Extracted UUID is %x\r\n", extracted_uuid.uuid);
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
												//APPL_LOG("Match Found 16\r\n");
                        return true;
                    }
                } else {
									//APPL_LOG("Unsuccessful Decode\r\n");
								}
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE)
                )
        {
						//printf("Decoding 32\r\n");
            for (uint32_t u_index = 0; u_index < (field_length/UUID32_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE, 
                &p_data[u_index * UUID32_SIZE + index + 2], 
                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
												//APPL_LOG("Match Found 32\r\n");
                        return true;
                    }
                }
            }
        }
        
        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
                )
        {
						//printf("Decoding 128\r\n");
            err_code = sd_ble_uuid_decode(UUID128_SIZE, 
                                          &p_data[index + 2], 
                                          &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
								//printf("Extracted UUID is %x\r\n", extracted_uuid.uuid);
                if ((extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
										//printf("Match Found 128");
                    return true;
                }
            }  else {
							//APPL_LOG("Decode Unsuccessful\r\n");
						}
        }
				else if (field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME) {
						//printf("Local Name Found\r\n");
						if ((p_data[index+2] == 0x48) && (p_data[index+3] == 0x50)){
							printf("BLE HPEH Service Found\r\n");
							return true;
						}     
				}
        index += field_length + 1;
    }
    return false;
}



/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */




static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
	{
		APPL_LOG("\nBLE Event Dispatch\r\n");
		switch (p_ble_evt->header.evt_id) {
			case BLE_GAP_EVT_CONNECTED:
				printf("BLE_GAP_EVT_CONNECTED\r\n");
			case BLE_GAP_EVT_DISCONNECTED:
				printf("BLE_GAP_EVT_DISCONNECTED\r\n");
			case BLE_GAP_EVT_CONN_PARAM_UPDATE:
				printf("BLE_GAP_EVT_CONN_PARAM_UPDATE Event\r\n");
			case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
				printf("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
			case BLE_GAP_EVT_SEC_INFO_REQUEST:
				printf("BLE_GAP_EVT_SEC_INFO_REQUEST\r\n");
			case BLE_GAP_EVT_PASSKEY_DISPLAY:
				printf("BLE_GAP_EVT_PASSKEY_DISPLAY\r\n");
			case BLE_GAP_EVT_KEY_PRESSED:
				printf("BLE_GAP_EVT_KEY_PRESSED\r\n");
			case BLE_GAP_EVT_AUTH_KEY_REQUEST:
				printf("BLE_GAP_EVT_AUTH_KEY_REQUEST\r\n");
			case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
				printf("BLE_GAP_EVT_LESC_DHKEY_REQUEST\r\n");
			case BLE_GAP_EVT_AUTH_STATUS:
				printf("BLE_GAP_EVT_AUTH_STATUS\r\n");
			case BLE_GAP_EVT_CONN_SEC_UPDATE:
				printf("BLE_GAP_EVT_CONN_SEC_UPDATE\r\n");
			case BLE_GAP_EVT_TIMEOUT:
				printf("BLE_GAP_EVT_TIMEOUT\r\n");
			case BLE_GAP_EVT_RSSI_CHANGED:
				printf("BLE_GAP_EVT_RSSI_CHANGED\r\n");
			case BLE_GAP_EVT_ADV_REPORT:
				printf("BLE_GAP_EVT_ADV_REPORT\r\n");
			case BLE_GAP_EVT_SEC_REQUEST:
				APPL_LOG("BLE_GAP_EVT_SEC_REQUEST\r\n");
			case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
				APPL_LOG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST\r\n");
			case BLE_GAP_EVT_SCAN_REQ_REPORT:
				APPL_LOG("BLE_GAP_EVT_SCAN_REQ_REPORT\r\n");
		}
			
		//printf("BLE Dispatch, Event Header is %x\r\n", p_ble_evt->header.evt_id);
    on_ble_evt(p_ble_evt);
		bsp_btn_ble_on_ble_evt(p_ble_evt);
		dm_ble_evt_handler(p_ble_evt);
		ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
		ble_our_service_c_on_ble_evt(&m_ble_our_service_c, p_ble_evt);

		//printf("Starting db_discovery\r\n");
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
	  //printf("Starting our_service\r\n");
    //printf("Starting BTN_BLE\r\n");

		//printf("Starting On_BLE_EVT\r\n");

		//printf("Finished with dispatch\r\n");
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
		
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
		
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APPL_LOG("Enable Params Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APPL_LOG("Softdevice Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APPL_LOG("Softdevice BLE Handler Set Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APPL_LOG("Softdevice Syst Handler Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);
		
		APPL_LOG("ble_stack_init complete\r\n");
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    err_code = pstorage_init();
		APPL_LOG("PStorage Error Code is %d\r\n",err_code);
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
		APPL_LOG("DM Init Error Code is %d\r\n",err_code);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof (ble_gap_sec_params_t));

    // Event handler to be registered with the module.
    register_param.evt_handler            = device_manager_event_handler;

    // Service or protocol context for device manager to load, store and apply on behalf of application.
    // Here set to client as application is a GATT client.
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_CLI_ID;

    // Secuirty parameters to be used for security procedures.
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.sec_param.kdist_peer.enc = 1;
    register_param.sec_param.kdist_peer.id  = 1;

    err_code = dm_register(&m_dm_app_id, &register_param);
    APPL_LOG("DM Register Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);
}


/**@brief Function for disabling the use of whitelist for scanning.
 */
static void whitelist_disable(void)
{
    uint32_t err_code;

    if ((m_scan_mode == BLE_WHITELIST_SCAN) && !m_whitelist_temporarily_disabled)
    {
        m_whitelist_temporarily_disabled = true;

        err_code = sd_ble_gap_scan_stop();
        if (err_code == NRF_SUCCESS)
        {
            scan_start();
        }
        else if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APPL_LOG("Whitelist Error Code is %d\r\n",err_code);
						APP_ERROR_CHECK(err_code);
        }
    }
    m_whitelist_temporarily_disabled = true;
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APPL_LOG("EVENT DISCONNECT Error Code is %d\r\n",err_code);
								APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            whitelist_disable();
            break;

        default:
            break;
    }
}


/**@brief Service Event Handler that runs whenever service event occurs.
 */
static void our_service_c_evt_handler(ble_our_service_c_t * p_our_service_c, ble_our_service_c_evt_t * p_our_service_c_evt)
{
		APPL_LOG("Our Service BLE Handler");
    uint32_t err_code;
    switch (p_our_service_c_evt->evt_type)
    {
        case BLE_OUR_SERVICE_C_EVT_DISCOVERY_COMPLETE:
				{		
						APPL_LOG("Service Discovery Complete");
            //Service discovered. Enable notification of measurements.
            err_code = ble_our_service_c_our_service_notif_enable(p_our_service_c);
            APPL_LOG("Our Service Notif Enable Error Code is %d\r\n",err_code);
						APP_ERROR_CHECK(err_code);
            APPL_LOG("Service discovered \r\n");
            break;
				}
        case BLE_OUR_SERVICE_C_TEMP_EVT_NOTIFICATION:
        {
            APPL_LOG("[APPL]: Temp Measurement received %d \r\n", p_our_service_c_evt->measurement);

            APPL_LOG("Temp = %d\r\n", p_our_service_c_evt->measurement);
            break;
        }
				
				case BLE_OUR_SERVICE_C_PRESS_EVT_NOTIFICATION:
        {
            APPL_LOG("[APPL]: Press Measurement received %d \r\n", p_our_service_c_evt->measurement);

            APPL_LOG("Press = %d\r\n", p_our_service_c_evt->measurement);
            break;
        }

        default:
            break;
    }
}


/**@brief Battery levelCollector Handler.
 */
/*
static void bas_c_evt_handler(ble_bas_c_t * p_bas_c, ble_bas_c_evt_t * p_bas_c_evt)
{
    uint32_t err_code;

    switch (p_bas_c_evt->evt_type)
    {
        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
            // Batttery service discovered. Enable notification of Battery Level.
            APPL_LOG_DEBUG("[APPL]: Battery Service discovered. \r\n");

            APPL_LOG_DEBUG("[APPL]: Reading battery level. \r\n");

            err_code = ble_bas_c_bl_read(p_bas_c);
            APP_ERROR_CHECK(err_code);


            APPL_LOG_DEBUG("[APPL]: Enabling Battery Level Notification. \r\n");
            err_code = ble_bas_c_bl_notif_enable(p_bas_c);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_BAS_C_EVT_BATT_NOTIFICATION:
        {
            APPL_LOG_DEBUG("[APPL]: Battery Level received %d %%\r\n", p_bas_c_evt->params.battery_level);

            APPL_LOG_DEBUG("Battery = %d %%\r\n", p_bas_c_evt->params.battery_level);
            break;
        }

        case BLE_BAS_C_EVT_BATT_READ_RESP:
        {
            APPL_LOG_DEBUG("[APPL]: Battery Level Read as %d %%\r\n", p_bas_c_evt->params.battery_level);

            APPL_LOG_DEBUG("Battery = %d %%\r\n", p_bas_c_evt->params.battery_level);
            break;
        }

        default:
            break;
    }
}
*/

/**
 * @brief Heart rate collector initialization.
 */
/*
static void hrs_c_init(void)
{
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler = hrs_c_evt_handler;

    uint32_t err_code = ble_hrs_c_init(&m_ble_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}
*/
static void our_service_c_init(void)
{
		APPL_LOG("Our Service C Init\r\n");
		//Creating Our Service C Init T Object
		//Just contains ble_our_service_c_evt_handler_t, event handler that runs whenever service event happens
		ble_our_service_c_init_t our_service_c_init_obj;
		//Places our_service_c_evt_handler in this object, function is defined above
    our_service_c_init_obj.evt_handler = our_service_c_evt_handler;
    uint32_t err_code = ble_our_service_c_init(&m_ble_our_service_c, &our_service_c_init_obj);
    APPL_LOG("Error Code is %d\r\n", err_code);
		APP_ERROR_CHECK(err_code);
}

/**
 * @brief Battery level collector initialization.
 */
/*
static void bas_c_init(void)
{
    ble_bas_c_init_t bas_c_init_obj;

    bas_c_init_obj.evt_handler = bas_c_evt_handler;

    uint32_t err_code = ble_bas_c_init(&m_ble_bas_c, &bas_c_init_obj);
    APP_ERROR_CHECK(err_code);
}
*/

/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
		//printf("Running db_discovery_init\r\n");
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
		APPL_LOG("DB Discovery Init Error Code is %d\r\n",err_code);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
		//printf("Scan Start\r\n");
    ble_gap_whitelist_t   whitelist;
    ble_gap_addr_t      * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    ble_gap_irk_t       * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];
    uint32_t              err_code;
    uint32_t              count;

    // Verify if there is any flash access pending, if yes delay starting scanning until 
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APPL_LOG("PStorage Access Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

    // Initialize whitelist parameters.
    whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
    whitelist.irk_count  = 0;
    whitelist.pp_addrs   = p_whitelist_addr;
    whitelist.pp_irks    = p_whitelist_irk;

    // Request creating of whitelist.
    err_code = dm_whitelist_create(&m_dm_app_id,&whitelist);
    APPL_LOG("WhiteList Create Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);

    if (((whitelist.addr_count == 0) && (whitelist.irk_count == 0)) ||
        (m_scan_mode != BLE_WHITELIST_SCAN)                        ||
        (m_whitelist_temporarily_disabled))
    {
        // No devices in whitelist, hence non selective performed.
        m_scan_param.active       = 0;            // Active scanning set.
        m_scan_param.selective    = 0;            // Selective scanning not set.
        m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
        m_scan_param.window       = SCAN_WINDOW;  // Scan window.
        m_scan_param.p_whitelist  = NULL;         // No whitelist provided.
        m_scan_param.timeout      = 0x0000;       // No timeout.
    }
    else
    {
        // Selective scanning based on whitelist first.
        m_scan_param.active       = 0;            // Active scanning set.
        m_scan_param.selective    = 1;            // Selective scanning not set.
        m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
        m_scan_param.window       = SCAN_WINDOW;  // Scan window.
        m_scan_param.p_whitelist  = &whitelist;   // Provide whitelist.
        m_scan_param.timeout      = 0x001E;       // 30 seconds timeout.
    }

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APPL_LOG("BLE GAP Scan Start Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
    APPL_LOG("BSP Indication Set Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);
		
		//APPL_LOG("Scan Start Ended\r\n");
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APPL_LOG("BSP Init Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APPL_LOG("BSP BTN BLE Init Error Code is %d\r\n",err_code);
		APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void nrf_log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT();
		APPL_LOG("NRF Log Error Code is %d\r\n",err_code);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
		APPL_LOG("Power Manage Error Code is %d\r\n",err_code);
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
		APPL_LOG("Starting Main\r\n");
    bool erase_bonds;

    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    buttons_leds_init(&erase_bonds);
    nrf_log_init();
    APPL_LOG("\r\n\n\nCentral HPEH Program\r\n");
		ble_stack_init();
    device_manager_init(erase_bonds);
    db_discovery_init();
    our_service_c_init();
		//APPL_LOG("Completed Init\r\n");


    // Start scanning for peripherals and initiate connection
    // with devices that advertise Heart Rate UUID.
    scan_start();

    for (;; )
    {
        power_manage();
    }
}


