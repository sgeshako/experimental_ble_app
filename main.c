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

/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
//#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "bsp.h"
#include "ble_gap.h"
#include "nrf_delay.h"
#include "spi.h"
#include "ble_ims.h"
#include "nrf_queue.h"
#include "ble_nus.h"
#include "filters.h"
#include "hid.h"
#include "hid_peer_manager.h"
#include "ble_advertising.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "pwm_adv_indication.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED_PIN             BSP_BOARD_LED_0                             /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN               BSP_BOARD_LED_1                             /**< Is on when device has connected. */

#define LEDBUTTON_LED_PIN               BSP_BOARD_LED_1                             /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_1                                /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "Nordic_Blinky"                             /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(30, UNIT_1_25_MS)             /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                    6                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(430, UNIT_10_MS)              /**< Connection supervisory timeout (430 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_lbs_t                        m_lbs;                                      /**< LED Button Service instance. */
static ble_ims_t                        m_ims;
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static ble_hids_t 						m_hids;                                   /**< Structure used to identify the HID service. */
static ble_bas_t  						m_bas;                                    /**< Structure used to identify the battery service. */


APP_TIMER_DEF(m_notification_timer_id);
APP_TIMER_DEF(m_sampling_timer_id);
static volatile uint16_t m_counter_value = 0;

#define SAMPLING_MS 5

#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(40, APP_TIMER_PRESCALER)
#define SAMPLING_INTERVAL               APP_TIMER_TICKS(SAMPLING_MS, APP_TIMER_PRESCALER)

#define M_PI		3.14159265358979323846

NRF_QUEUE_DEF(sample_combo_t, m_sample_queue, 20, NRF_QUEUE_MODE_NO_OVERFLOW);

static const double dt = SAMPLING_MS * 0.001f;
static volatile double gyro_x_raw_angle = 0;
static volatile double gyro_x_angle = 0;
static double gyro_x_angle_mahony = 0;
static double gyro_x_angle_kalman = 0;
static volatile double acc_x_angle = 0;
static double acc_y_angle = 0;

#define LEFT_KEY 0x50 // Keyboard Left Arrow
#define RIGHT_KEY 0x4f // Keyboard Right Arrow
#define KEY_RELEASE 0x00 // Empty code signifies key release
static bool m_key_last_state_pressed = true;

// Box flag in array to prevent some weird compiler optimization.
static uint8_t is_advertising[1] = {0};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // app_timer_create can come here
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
//static void advertising_init(void)
//{
//    uint32_t      err_code;
//    ble_advdata_t advdata;
//    ble_advdata_t scanrsp;
//
//    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};
//
//    // Build and set advertising data
//    memset(&advdata, 0, sizeof(advdata));
//
//    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
//    advdata.include_appearance = true;
//    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
//
//
//    memset(&scanrsp, 0, sizeof(scanrsp));
//    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
//    scanrsp.uuids_complete.p_uuids  = adv_uuids;
//
//    err_code = ble_advdata_set(&advdata, &scanrsp);
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED_PIN);
        NRF_LOG_INFO("Received LED ON!\r\n");
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED_PIN);
        NRF_LOG_INFO("Received LED OFF!\r\n");
    }
}

static void timer_write_handler(ble_ims_t * p_lbs, uint8_t timer_state)
{
	uint32_t err_code;
	uint8_t start_sampling = timer_state & 0xF0;
	uint8_t start_notifying = timer_state & 0x0F;

	if (start_sampling)
	{
		err_code = app_timer_start(m_sampling_timer_id, SAMPLING_INTERVAL, NULL);
		if (err_code != NRF_SUCCESS)
		{
			NRF_LOG_INFO("Sampling timer failed: %d", err_code);
		}
	}
	else
	{
		err_code = app_timer_stop(m_sampling_timer_id);
		gyro_x_raw_angle = 0;
		acc_x_angle = 0;
		gyro_x_angle = 0;
	}
	APP_ERROR_CHECK(err_code);

	if (start_notifying)
	{
	    err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
		if (err_code != NRF_SUCCESS)
		{
			NRF_LOG_INFO("Notification timer failed: %d", err_code);
		}
	}
    else
    {
		err_code = app_timer_stop(m_notification_timer_id);
    }
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_lbs_init_t init;

    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);

    ble_ims_init_t ims_init;
    ims_init.timer_write_handler = timer_write_handler;

    err_code = ble_ims_init(&m_ims, &ims_init);
    APP_ERROR_CHECK(err_code);


	ble_nus_init_t nus_init;
	nus_init.data_handler = NULL; // No handling of incoming data

	err_code = ble_nus_init(&m_nus, &nus_init);
	APP_ERROR_CHECK(err_code);

	hid_services_init(&m_hids, &m_bas);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
//static void advertising_start(void)
//{
//    uint32_t             err_code;
//    ble_gap_adv_params_t adv_params;
//
//    // Start advertising
//    memset(&adv_params, 0, sizeof(adv_params));
//
//    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
//    adv_params.p_peer_addr = NULL;
//    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
//    adv_params.interval    = APP_ADV_INTERVAL;
//    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
//
//    err_code = sd_ble_gap_adv_start(&adv_params);
//    APP_ERROR_CHECK(err_code);
//    bsp_board_led_on(ADVERTISING_LED_PIN);
////    NRF_LOG_INFO("Advertising started...\r\n");
//}

static void set_advertising(bool active);

/**@brief Function for handling the Application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_CONNECTED:
		    NRF_LOG_INFO("BLE evt: BLE_GAP_EVT_CONNECTED\r\n");
			break;
		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
			NRF_LOG_INFO("BLE evt: BLE_GAP_EVT_CONN_PARAM_UPDATE\r\n");
			break;
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			NRF_LOG_INFO("BLE evt: BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
			break;
		case BLE_GAP_EVT_CONN_SEC_UPDATE:
			NRF_LOG_INFO("BLE evt: BLE_GAP_EVT_CONN_SEC_UPDATE\r\n");
			break;
		case BLE_GAP_EVT_AUTH_STATUS:
			NRF_LOG_INFO("BLE evt: BLE_GAP_EVT_AUTH_STATUS\r\n");
			break;
		case BLE_GATTS_EVT_WRITE:
			NRF_LOG_INFO("BLE evt: BLE_GATTS_EVT_WRITE\r\n");
			break;
		default:
			NRF_LOG_INFO("BLE evt: %d\r\n", p_ble_evt->header.evt_id);
			break;
	}

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//            NRF_LOG_INFO("Connected\r\n");
//            bsp_board_led_on(CONNECTED_LED_PIN); // Commented when using PWM
        	set_advertising(false);
            bsp_board_led_off(ADVERTISING_LED_PIN);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_EVT_TX_COMPLETE:
            // Send next key event
            (void) buffer_dequeue(true);
            break; // BLE_EVT_TX_COMPLETE

        case BLE_GAP_EVT_DISCONNECTED:
//            NRF_LOG_INFO("Disconnected\r\n");
//            bsp_board_led_off(CONNECTED_LED_PIN); // Commented when using PWM
        	// Dequeue all keys without transmission.
			(void) buffer_dequeue(false);
			hid_reset_caps_lock_state();
			// Can't do, throws error.
//			hid_pm_update_whitelist();
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            set_advertising(true); // Timeout and disconnect restart advertising
//            advertising_start(); // Peer Manager starts advertising automatically
            break; // BLE_GAP_EVT_DISCONNECTED
        case BLE_GAP_EVT_TIMEOUT:
        	if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
        	{
				set_advertising(true);
			}
        	break; // BLE_GAP_EVT_TIMEOUT

//        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
//            // Pairing not supported
//            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
//                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
//                                                   NULL,
//                                                   NULL);
//            APP_ERROR_CHECK(err_code);
//            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	/** The Connection state module has to be fed BLE events in order to function correctly
	 * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
	ble_conn_state_on_ble_evt(p_ble_evt);
	hid_pm_on_ble_evt(p_ble_evt);
	ble_advertising_on_ble_evt(p_ble_evt);

    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_lbs_on_ble_evt(&m_lbs, p_ble_evt);
    ble_ims_on_ble_evt(&m_ims, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    ble_hids_on_ble_evt(&m_hids, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
}


/**@brief   Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
	NRF_LOG_INFO("Sys evt: %d\r\n", sys_evt);
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
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
    APP_ERROR_CHECK(err_code);
    ble_enable_params.common_enable_params.vs_uuid_count = 2;

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
//    uint32_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
            NRF_LOG_INFO("Send button state change.\r\n");            
//            err_code = ble_lbs_on_button_change(&m_lbs, button_action);
//            if (err_code != NRF_SUCCESS &&
//                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//                err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
            send_sample_string_to_peer();
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

static void sampling_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    read_sensor_non_blocking();
}

static void notification_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    m_counter_value++;

	uint32_t err_code;

    err_code = ble_ims_on_counter_change(&m_ims, m_counter_value);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("BLE counter change failed: %d", err_code);
	}
//    APP_ERROR_CHECK(err_code);
}

static void set_advertising(bool active)
{
	is_advertising[0] = active;
}

static bool is_advertising_active()
{
	return is_advertising[0] != 0;
}


/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    
    // Initialize.
    leds_init();
    timers_init();

    err_code = app_timer_create(&m_sampling_timer_id, APP_TIMER_MODE_REPEATED, sampling_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_notification_timer_id, APP_TIMER_MODE_REPEATED, notification_timeout_handler);
    APP_ERROR_CHECK(err_code);

    pwm_adv_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Initializing...\r\n");

    buttons_init();
    ble_stack_init();
    peer_manager_init(false);
    gap_params_init();
    services_init();
    advertising_init_with_peer_manager(set_advertising);
    conn_params_init();
    hid_buffer_init();

    // Start execution.
    NRF_LOG_INFO("Blinky Start!\r\n");
    advertising_start_with_peer_manager();
    set_advertising(true);

    configure(&m_sample_queue);

    // Enter main loop.
    for (;;)
    {
    	if (nrf_queue_is_full(&m_sample_queue))
    	{
			int i = 0;
			sample_combo_t data[2];
			memset(data, 0, sizeof(data));

			while (nrf_queue_pop(&m_sample_queue, &data[i++ % 2]) == NRF_SUCCESS)
			{
				int16_t acc_x = data[(i-1)%2].acc.x.conv;
				int16_t acc_y = data[(i-1)%2].acc.y.conv;
				int16_t acc_z = data[(i-1)%2].acc.z.conv;

				int16_t gyro_x_rate = data[(i-1)%2].gyro.x.conv;

				gyro_x_raw_angle += gyro_x_rate * 0.00875f * dt;

				if (!(acc_x == 0 && acc_y == 0 && acc_z == 0 && gyro_x_rate == 0))
				{
					if (acc_z != 0)
					{
						acc_x_angle = -atan((double)acc_x / acc_z) * 180.0 / M_PI;
						acc_y_angle = -atan((double)acc_y / acc_z) * 180.0 / M_PI;
					}

					gyro_x_angle = comp_filter(gyro_x_rate, acc_y_angle, dt);
					gyro_x_angle_mahony = mahony_filter(gyro_x_rate, acc_y_angle, dt);
					gyro_x_angle_kalman = kalman_filter(gyro_x_rate, acc_y_angle, dt);
				}

//				NRF_LOG_RAW_INFO("Complementary:"NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyro_y_angle));
//				NRF_LOG_RAW_INFO("\t");
//				NRF_LOG_RAW_INFO("Mahony:"NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyro_y_angle_mahony));
//				NRF_LOG_RAW_INFO("\t");
//				NRF_LOG_RAW_INFO("Kalman:"NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyro_y_angle_kalman));
//				NRF_LOG_RAW_INFO("\r\n")
			}
			NRF_LOG_RAW_INFO("Calculated angle [100ms]: "NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyro_x_angle));
			NRF_LOG_RAW_INFO("\r\n");

			// Send Left/Right arrow key notification when angle goes above or below 25 degrees
			if (!m_key_last_state_pressed && gyro_x_angle > 25)
			{
					err_code = send_key_scan_code(LEFT_KEY);
					m_key_last_state_pressed = true;
			}
			else if (!m_key_last_state_pressed && gyro_x_angle < -25)
			{
				err_code = send_key_scan_code(RIGHT_KEY);
				m_key_last_state_pressed = true;
			}
			else if (m_key_last_state_pressed && !(gyro_x_angle > 25 || gyro_x_angle < -25))
			{
					err_code = send_key_scan_code(KEY_RELEASE);
					m_key_last_state_pressed = false;
			}
			if (err_code != NRF_SUCCESS) {
				NRF_LOG_INFO("Key notification failed %d\r\n", err_code);
			}

			NRF_LOG_FLUSH();
		}

//        bsp_board_led_invert(LEDBUTTON_LED_PIN);
//        nrf_delay_ms(1000);

    	bool emptyLogBuffer = !NRF_LOG_PROCESS();

        if (is_advertising_active())
        {
			run_pwm_adv_indication();
		}
        else if (emptyLogBuffer)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
