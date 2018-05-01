
#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "our_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "ble.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "sdk_common.h"

// ALREADY_DONE_FOR_YOU: Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_our_service_on_ble_evt(ble_os_t * p_our_service, ble_evt_t * p_ble_evt)
{
    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
		switch (p_ble_evt->header.evt_id)
		{
				case BLE_GAP_EVT_CONNECTED:
						p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						break;
				case BLE_GAP_EVT_DISCONNECTED:
						p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
						break;
				default:
						// No implementation needed.
						break;
		}
}

void ble_customservice_on_db_disc_evt(ble_os_t * p_os_c, ble_db_discovery_evt_t * p_evt)
{
    ble_custom_evt_t custom_evt;
    memset(&custom_evt,0,sizeof(ble_custom_evt_t));

    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the NUS was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_OUR_SERVICE_UUID)
/*		&&
        p_evt->params.discovered_db.srv_uuid.type == p_ble_nus_c->uuid_type)
*/
    {
        uint32_t i;
        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case BLE_UUID_TEMP_CHARACTERISTIC:
                    custom_evt.handles.custom_tx_handle = p_chars[i].characteristic.handle_value;
                    break;

                case BLE_UUID_PRESS_CHARACTERISTIC:
                    custom_evt.handles.custom_rx_handle = p_chars[i].characteristic.handle_value;
                    custom_evt.handles.custom_rx_cccd_handle = p_chars[i].cccd_handle;
                    break;

                default:
                    break;
            }
        }
    }
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t our_char_add(ble_os_t * p_our_service)
{
    // OUR_JOB: Step 2.A, Add temperature characteristic UUID
    uint32_t            temp_err_code;
		ble_uuid_t          temp_char_uuid;
		ble_uuid128_t       base_uuid = BLE_UUID_OUR_BASE_UUID;
		temp_char_uuid.uuid      = BLE_UUID_TEMP_CHARACTERISTIC;
		temp_err_code = sd_ble_uuid_vs_add(&base_uuid, &temp_char_uuid.type);
		APP_ERROR_CHECK(temp_err_code);
	
	  // OUR_JOB: Step 2.A, Add pressure characteristic UUID
    uint32_t            press_err_code;
		ble_uuid_t          press_char_uuid;
		press_char_uuid.uuid      = BLE_UUID_PRESS_CHARACTERISTIC;
		press_err_code = sd_ble_uuid_vs_add(&base_uuid, &press_char_uuid.type);
		APP_ERROR_CHECK(press_err_code);
	
    // OUR_JOB: Step 2.F Add read/write properties to temperature characteristic
    ble_gatts_char_md_t temp_char_md;
    memset(&temp_char_md, 0, sizeof(temp_char_md));
		temp_char_md.char_props.read = 1;
		//char_md.char_props.write = 1;
	
	  // OUR_JOB: Step 2.F Add read/write properties to pressure characteristic
    ble_gatts_char_md_t press_char_md;
    memset(&press_char_md, 0, sizeof(press_char_md));
		press_char_md.char_props.read = 1;
		//char_md.char_props.write = 1;
    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t temp_cccd_md;
    memset(&temp_cccd_md, 0, sizeof(temp_cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_cccd_md.write_perm);
		temp_cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
		temp_char_md.p_cccd_md           = &temp_cccd_md;
		temp_char_md.char_props.notify   = 1;
		
		// OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
		ble_gatts_attr_md_t press_cccd_md;
    memset(&press_cccd_md, 0, sizeof(press_cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&press_cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&press_cccd_md.write_perm);
		press_cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
		press_char_md.p_cccd_md           = &press_cccd_md;
		press_char_md.char_props.notify   = 1;
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t temp_attr_md;
    memset(&temp_attr_md, 0, sizeof(temp_attr_md));  
    temp_attr_md.vloc        = BLE_GATTS_VLOC_STACK;
    
		// OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t press_attr_md;
    memset(&press_attr_md, 0, sizeof(press_attr_md));  
    press_attr_md.vloc        = BLE_GATTS_VLOC_STACK;
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_attr_md.write_perm);
  
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&press_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&press_attr_md.write_perm);
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    temp_attr_char_value;
		memset(&temp_attr_char_value, 0, sizeof(temp_attr_char_value));    
		temp_attr_char_value.p_uuid      = &temp_char_uuid;
		temp_attr_char_value.p_attr_md   = &temp_attr_md;
 
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    press_attr_char_value;
		memset(&press_attr_char_value, 0, sizeof(press_attr_char_value));    
		press_attr_char_value.p_uuid      = &press_char_uuid;
		press_attr_char_value.p_attr_md   = &press_attr_md;
  
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
		temp_attr_char_value.max_len     = 4;
		temp_attr_char_value.init_len    = 4;
		//uint8_t value[4]            = {0x12,0x34,0x56,0x78};
		//temp_attr_char_value.p_value     = value;
		
		press_attr_char_value.max_len     = 4;
		press_attr_char_value.init_len    = 4;
		//uint8_t value[4]            = {0x12,0x34,0x56,0x78};
		//press_attr_char_value.p_value     = value;
				
    // OUR_JOB: Step 2.E, Add our new characteristic to the service
		temp_err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &temp_char_md,
                                   &temp_attr_char_value,
                                   &p_our_service->temp_char_handles);
		APP_ERROR_CHECK(temp_err_code);

		// OUR_JOB: Step 2.E, Add our new characteristic to the service
		press_err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &press_char_md,
                                   &press_attr_char_value,
                                   &p_our_service->press_char_handles);
		APP_ERROR_CHECK(press_err_code);
		
    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void our_service_init(ble_os_t * p_our_service)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // FROM_SERVICE_TUTORIAL: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
	  //BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_OUR_SERVICE_UUID);
    APP_ERROR_CHECK(err_code);    
    
    // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
		p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
	
    // FROM_SERVICE_TUTORIAL: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_our_service->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    our_char_add(p_our_service);
}

// ALREADY_DONE_FOR_YOU: Function to be called when updating characteristic value
void temperature_characteristic_update(ble_os_t *p_our_service, int32_t *temperature_value)
{
    // OUR_JOB: Step 3.E, Update characteristic value
		if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				uint16_t               len = 4;
				ble_gatts_hvx_params_t hvx_params;
				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_our_service->temp_char_handles.value_handle;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = 0;
				hvx_params.p_len  = &len;
				hvx_params.p_data = (uint8_t*)temperature_value;  

				sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
		}
}

void pressure_characteristic_update(ble_os_t *p_our_service, int32_t *pressure_value)
{
    // OUR_JOB: Step 3.E, Update characteristic value
		if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				uint16_t               len = 4;
				ble_gatts_hvx_params_t hvx_params;
				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_our_service->press_char_handles.value_handle;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = 0;
				hvx_params.p_len  = &len;
				hvx_params.p_data = (uint8_t*)pressure_value;  

				sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
		}
}
