#include <stdint.h>
#include <stdbool.h>

typedef enum {
	PWM_ADV_INDICATE_DIRECTED_ADVERTISING,
	PWM_ADV_INDICATE_FAST_ADVERTISING,
	PWM_ADV_INDICATE_SLOW_ADVERTISING,
	PWM_ADV_INDICATE_FAST_WHITELIST_ADVERTISING,
	PWM_ADV_INDICATE_SLOW_WHITELIST_ADVERTISING,
	PWM_ADV_INDICATE_IDLE
} pwm_adv_mode_t;

typedef struct {
	uint32_t current_count;      /**< Current count of delay. */
	uint16_t max_count;          /**< Maximum count before delay expires. */
	uint8_t  duty_cycle_step;    /**< Current active duty cycle step. */
	bool direction_up;           /**< True is step direction UP, False is DOWN. */
	bool direction_mode_reverse; /**< True is step reverses at bounds, False is starts back from beginning. */

} pwm_adv_duty_cycle_config;

/**@brief Initialize and enable PWM instance.
 */
void pwm_adv_init(void);

/**@brief Dims a LED in different patterns to indicate the currently active advertising mode.
 *
 * @details Every N number of invocations will update the PWM duty cycle to apply a given LED dimming pattern.
 */
void run_pwm_adv_indication();

/**@brief Set LED indication pattern to the currently active advertising mode.
 */
void set_pwm_adv_mode(pwm_adv_mode_t adv_mode);
