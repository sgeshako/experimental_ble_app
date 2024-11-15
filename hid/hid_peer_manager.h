#include "peer_manager.h"

/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size);

/**@brief Handler callback for setting Advertising state in main. */
typedef void (*hid_pm_set_advertising_handler_t)(bool active);

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init_with_peer_manager(hid_pm_set_advertising_handler_t set_advertising);

/**@brief Function for starting advertising with Peer Manager.
 */
void advertising_start_with_peer_manager(void);

/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
void peer_manager_init(bool erase_bonds);

void hid_pm_update_whitelist(void);

/**@brief Wrapper call to Peer Manager function.
 */
void hid_pm_on_ble_evt(ble_evt_t * p_ble_evt);
