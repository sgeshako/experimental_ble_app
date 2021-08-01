/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 *
 * @brief Inertial Module Service module.
 *
 * @details This module implements a custom LED Button Service with an LED and Button Characteristics.
 *          During initialization, the module adds the LED Button Service and Characteristics
 *          to the BLE stack database.
 *
 *          The application must supply an event handler for receiving LED Button Service
 *          events. Using this handler, the service notifies the application when the
 *          LED value changes.
 *
 *          The service also provides a function for letting the application notify
 *          the state of the Button Characteristic to connected peers.
 *
 * @note The application must propagate BLE stack events to the LED Button Service
 *       module by calling ble_lbs_on_ble_evt() from the @ref softdevice_handler callback.
*/

#ifndef BLE_IMS_H__
#define BLE_IMS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_lbs.h"

#define IMS_UUID_BASE LBS_UUID_BASE
#define IMS_UUID_SERVICE     0x2523
#define IMS_UUID_COUNTER_CHAR 0x2524
#define IMS_UUID_TIMER_CHAR 0x2525
#define IMS_UUID_SAMPLE_CHAR 0x2526

// Forward declaration of the ble_ims_t type. 
typedef struct ble_ims_s ble_ims_t;

typedef void (*ble_ims_timer_write_handler_t) (ble_ims_t * p_ims, uint8_t new_state);

/** @brief LED Button Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_ims_timer_write_handler_t timer_write_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_ims_init_t;

/**@brief LED Button Service structure. This structure contains various status information for the service. */
struct ble_ims_s
{
    uint16_t                    service_handle;      /**< Handle of LED Button Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    counter_char_handles; /**< Handles related to the Button Characteristic. */
	  ble_gatts_char_handles_t    timer_char_handles;
		ble_gatts_char_handles_t    sample_char_handles;
    uint8_t                     uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_ims_timer_write_handler_t timer_write_handler;   /**< Event handler to be called when the LED Characteristic is written. */
};

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_lbs      LED Button Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_lbs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_ims_init(ble_ims_t * p_ims, const ble_ims_init_t * p_ims_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the LED Button Service.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_ims_on_ble_evt(ble_ims_t * p_ims, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_ims_on_counter_change(ble_ims_t * p_ims, uint16_t counter_value);

/**@brief Function for sending a button state notification.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_ims_on_sample_change(ble_ims_t * p_ims, uint8_t * p_sample, uint16_t sample_length);

#endif // BLE_IMS_H__

/** @} */
