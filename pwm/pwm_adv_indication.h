#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint32_t current_count;
	uint32_t max_count;
	uint8_t  duty_cycle_step;
	bool direction_up;       /**< True is step direction UP, False is DOWN. */
} pwm_adv_duty_cycle_config;

/**@brief Initialize and enable PWM instance.
 */
void pwm_adv_init(void);

/**@brief Dims a LED to indicate advertising is running.
 *
 * @details Every N number of invocations will update the PWM duty cycle to mimic a gradual LED dim.
 */
void run_pwm_adv_indication();
