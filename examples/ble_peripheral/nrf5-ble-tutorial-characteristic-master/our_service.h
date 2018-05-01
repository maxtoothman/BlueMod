
#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"


// FROM_SERVICE_TUTORIAL: Defining 16-bit service and 128-bit base UUIDs
#define BLE_UUID_OUR_BASE_UUID              {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define BLE_UUID_OUR_SERVICE_UUID                0xF00D // Just a random, but recognizable value
#define BLE_UUID_TEMP_CHARACTERISTIC 						 0xBEEF // Just a random, but recognizable value
#define BLE_UUID_PRESS_CHARACTERISTIC						 0xABCE

// ALREADY_DONE_FOR_YOU: Defining 16-bit characteristic UUID

// This structure contains various status information for our service. 
// The name is based on the naming convention used in Nordics SDKs. 
// 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
// ‘os’ is short for Our Service). 

typedef enum 
{
    BLE_CUSTOM_EVT_DISCOVERY_COMPLETE = 1, /**< Event indicating that the NUS service and its characteristics was found. */
    BLE_CUSTOM_EVT_NUS_RX_EVT,             /**< Event indicating that the central has received something from a peer. */
    BLE_CUSTOM_EVT_DISCONNECTED            /**< Event indicating that the NUS server has disconnected. */
} ble_custom_evt_type_t;

typedef struct
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    // OUR_JOB: Step 2.D, Add handles for the characteristic attributes to our struct
		ble_gatts_char_handles_t    temp_char_handles;
		ble_gatts_char_handles_t		press_char_handles;
}ble_os_t;

typedef struct {
	  uint16_t                		custom_rx_handle;      /**< Handle of the NUS RX characteristic as provided by a discovery. */
    uint16_t                		custom_rx_cccd_handle; /**< Handle of the CCCD of the NUS RX characteristic as provided by a discovery. */
    uint16_t                		custom_tx_handle;      /**< Handle of the NUS TX characteristic as provided by a discovery. */
} ble_custom_handles_t;

typedef struct {
    ble_custom_evt_type_t evt_type;
    uint16_t             conn_handle; 
    uint8_t            * p_data;
    uint8_t              data_len;
    ble_custom_handles_t   handles;     
} ble_custom_evt_t;

typedef struct ble_nus_c_s ble_nus_c_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_our_service_on_ble_evt(ble_os_t * p_our_service, ble_evt_t * p_ble_evt);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void our_service_init(ble_os_t * p_our_service);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void temperature_characteristic_update(ble_os_t *p_our_service, int32_t *temperature_value);

void pressure_characteristic_update(ble_os_t *p_our_service, int32_t *pressure_value);

#endif  /* _ OUR_SERVICE_H__ */
