/*
 * Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/**@file
 *
 * @defgroup ble_sdk_srv_hrs_c   Heart Rate Service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Heart Rate Service Client module.
 *
 * @details  This module contains the APIs and types exposed by the Heart Rate Service Client
 *           module. These APIs and types can be used by the application to perform discovery of
 *           Heart Rate Service at the peer and interact with it.
 *
 * @warning  Currently this module only has support for Heart Rate Measurement characteristic. This
 *           means that it will be able to enable notification of the characteristic at the peer and
 *           be able to receive Heart Rate Measurement notifications from the peer. It does not
 *           support the Body Sensor Location and the Heart Rate Control Point characteristics.
 *           When a Heart Rate Measurement is received, this module will decode only the
 *           Heart Rate Measurement Value (both 8 bit and 16 bit) field from it and provide it to
 *           the application.
 *
 * @note     The application must propagate BLE stack events to this module by calling
 *           ble_hrs_c_on_ble_evt().
 *
 */

#ifndef BLE_OUR_SERVICE_C_H__
#define BLE_OUR_SERVICE_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"

#define BLE_UUID_OUR_BASE_UUID              {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define BLE_UUID_OUR_SERVICE_UUID                0xF00D // Just a random, but recognizable value
#define BLE_UUID_TEMP_CHARACTERISTIC 						 0xBEEF // Just a random, but recognizable value
#define BLE_UUID_PRESS_CHARACTERISTIC						 0xABCE

/**
 * @defgroup hrs_c_enums Enumerations
 * @{
 */

/**@brief HRS Client event type. */
typedef enum
{
    BLE_OUR_SERVICE_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the Heart Rate Service has been discovered at the peer. */
    BLE_OUR_SERVICE_C_TEMP_EVT_NOTIFICATION,         /**< Event indicating that a notification of the Heart Rate Measurement characteristic has been received from the peer. */
		BLE_OUR_SERVICE_C_PRESS_EVT_NOTIFICATION,
		BLE_OUR_SERVICE_C_EVT_DISCONNECTED
} ble_our_service_c_evt_type_t;

/** @} */

/**
 * @defgroup hrs_c_structs Structures
 * @{
 */

/**@brief Structure containing the heart rate measurement received from the peer. */
typedef struct
{
    uint16_t temp_value;  /**< Heart Rate Value. */
		uint16_t press_value;
} ble_our_service_t;


/**@brief Structure containing the handles related to the Heart Rate Service found on the peer. */
typedef struct
{
    uint16_t our_service_cccd_handle;  /**< Handle of the CCCD of the Heart Rate Measurement characteristic. */
    uint16_t temp_handle;       /**< Handle of the Heart Rate Measurement characteristic as provided by the SoftDevice. */
		uint16_t press_handle;
} ble_our_service_c_handles_t;


/**@brief Temperature and Pressure Event structure. */
typedef struct
{
    ble_our_service_c_evt_type_t 							evt_type;    /**< Type of the event. */
    uint16_t             											conn_handle; /**< Connection handle on which the Heart Rate service was discovered on the peer device..*/
		ble_our_service_c_handles_t 							handles;
	  const uint16_t*													  measurement;	
} ble_our_service_c_evt_t;


//typedef struct
//{
//    ble_our_service_c_evt_type_t 							evt_type;    /**< Type of the event. */
//    uint16_t             											conn_handle; /**< Connection handle on which the Heart Rate service was discovered on the peer device..*/
//		ble_our_service_c_handles_t 							handles;
//	  uint16_t*																	temperature;
//} ble_our_service_c_temp_evt_t;


/** @} */

/**
 * @defgroup hrs_c_types Types
 * @{
 */

// Forward declaration of the ble_bas_t type.
typedef struct ble_our_service_c_s ble_our_service_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_our_service_c_evt_handler_t) (ble_our_service_c_t * p_ble_our_service_c, ble_our_service_c_evt_t * p_evt);

/** @} */

/**
 * @addtogroup hrs_c_structs
 * @{
 */

/**@brief Heart Rate Client structure.
 */
struct ble_our_service_c_s
{
		uint8_t																		 uuid_type;
    uint16_t                									 conn_handle;      /**< Connection handle as provided by the SoftDevice. */
    ble_our_service_c_handles_t                peer_our_service_db;      /**< Handles related to HRS on the peer*/
    ble_our_service_c_evt_handler_t 					 evt_handler;      /**< Application event handler to be called when there is an event related to the heart rate service. */
};

/**@brief Heart Rate Client initialization structure.
 */
typedef struct
{
    ble_our_service_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Heart Rate Client module whenever there is an event related to the Heart Rate Service. */
} ble_our_service_c_init_t;

/** @} */

/**
 * @defgroup hrs_c_functions Functions
 * @{
 */

/**@brief     Function for initializing the heart rate client module.
 *
 * @details   This function will register with the DB Discovery module. There it
 *            registers for the Heart Rate Service. Doing so will make the DB Discovery
 *            module look for the presence of a Heart Rate Service instance at the peer when a
 *            discovery is started.
 *
 * @param[in] p_ble_hrs_c      Pointer to the heart rate client structure.
 * @param[in] p_ble_hrs_c_init Pointer to the heart rate initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_our_service_c_init(ble_our_service_c_t * p_ble_our_service_c, ble_our_service_c_init_t * p_ble_our_service_c_init);

/**@brief Function for handling events from the database discovery module.
 *
 * @details This function will handle an event from the database discovery module, and determine
 *          if it relates to the discovery of NUS at the peer. If so, it will
 *          call the application's event handler indicating that NUS has been
 *          discovered at the peer. It also populates the event with the service related
 *          information before providing it to the application.
 *
 * @param[in] p_ble_nus_c Pointer to the NUS client structure.
 * @param[in] p_evt       Pointer to the event received from the database discovery module.
 */
 void ble_our_service_c_on_db_disc_evt(ble_our_service_c_t * p_ble_our_service_c, const ble_db_discovery_evt_t * p_evt);
 
/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function will handle the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the Heart Rate Client module, then it uses it to update
 *            interval variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_hrs_c Pointer to the heart rate client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event.
 */
void ble_our_service_c_on_ble_evt(ble_our_service_c_t * p_ble_our_service_c, const ble_evt_t * p_ble_evt);


/**@brief   Function for requesting the peer to start sending notification of Heart Rate
 *          Measurement.
 *
 * @details This function will enable to notification of the Heart Rate Measurement at the peer
 *          by writing to the CCCD of the Heart Rate Measurement Characteristic.
 *
 * @param   p_ble_hrs_c Pointer to the heart rate client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned 
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
uint32_t ble_our_service_c_our_service_notif_enable(ble_our_service_c_t * p_ble_our_service_c);


/**@brief     Function for handling events from the database discovery module.
 *
 * @details   Call this function when getting a callback event from the DB discovery modue.
 *            This function will handle an event from the database discovery module, and determine
 *            if it relates to the discovery of heart rate service at the peer. If so, it will
 *            call the application's event handler indicating that the heart rate service has been
 *            discovered at the peer. It also populates the event with the service related
 *            information before providing it to the application.
 *
 * @param[in] p_ble_hrs_c Pointer to the heart rate client structure instance to associate.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 *
 */
void ble_our_service_on_db_disc_evt(ble_our_service_c_t * p_ble_our_service_c, const ble_db_discovery_evt_t * p_evt);


/**@brief     Function for assigning a handles to a this instance of hrs_c.
 *
 * @details   Call this function when a link has been established with a peer to
 *            associate this link to this instance of the module. This makes it
 *            possible to handle several link and associate each link to a particular
 *            instance of this module.The connection handle and attribute handles will be
 *            provided from the discovery event @ref BLE_HRS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_hrs_c        Pointer to the heart rate client structure instance to associate.
 * @param[in] conn_handle        Connection handle to associated with the given Heart Rate Client Instance.
 * @param[in] p_peer_hrs_handles Attribute handles for the HRS server you want this HRS_C client to
 *                               interact with.
 */
uint32_t ble_our_service_c_handles_assign(ble_our_service_c_t *    p_ble_hrs_c,
                                  uint16_t         conn_handle,
                                  const ble_our_service_c_handles_t * p_peer_our_service_handles);

/** @} */ // End tag for Function group.

#endif // BLE_HRS_C_H__

/** @} */ // End tag for the file.
