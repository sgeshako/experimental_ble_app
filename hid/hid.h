#include <stdint.h>
#include "ble_hids.h"
#include "ble_bas.h"

/**@brief Function for initializing services that will be used by the application.
 */
void hid_services_init(ble_hids_t * p_hids, ble_bas_t * p_bas);

/**@brief   Function for initializing the buffer queue used to key events that could not be
 *          transmitted
 *
 * @warning This handler is an example only. You need to analyze how you wish to buffer or buffer at
 *          all.
 *
 * @note    In case of HID keyboard, a temporary buffering could be employed to handle scenarios
 *          where encryption is not yet enabled or there was a momentary link loss or there were no
 *          Transmit buffers.
 */
void hid_buffer_init(void);

/**@brief   Function to dequeue key scan patterns that could not be transmitted either completely of
 *          partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  tx_flag   Indicative of whether the dequeue should result in transmission or not.
 * @note       A typical example when all keys are dequeued with transmission is when link is
 *             disconnected.
 *
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
 */
uint32_t buffer_dequeue(bool tx_flag);

void send_sample_string_to_peer(void);

void hid_reset_caps_lock_state();

/**@brief Function for sending sample key presses to the peer.
 *
 * @param[in]   key_pattern_len   Pattern length.
 * @param[in]   p_key_pattern     Pattern to be sent.
 */
void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern);

/**@brief Function for transmitting a single key scan code Notification.
 *
 * @param[in]  key_scan_code   Key scan code to be sent.
 * @return     NRF_SUCCESS on success, BLE_ERROR_NO_TX_PACKETS in case transmission could not be
 *             completed due to lack of transmission buffer or other error codes indicating reason
 *             for failure.
 */
uint32_t send_key_scan_code(uint8_t   key_scan_code);
