#include "pwm_adv_indication.h"
#include "app_pwm.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_log.h"

#define ADVERTISING_LED_PIN             BSP_LED_1                             /**< Pin number for LED on board. */

// Create the instance "PWM1" using TIMER1.
APP_PWM_INSTANCE(PWM1,1);

/*
 * Pre-computed duty cycles after gamma correction
 */
static const uint16_t duty_cycle_sequence[] = { 0, 1, 2, 3, 5, 7, 10, 13, 17,
		22, 27, 33, 39, 46, 53, 61, 70, 79, 89, 100 };

static const uint8_t sequence_size = sizeof(duty_cycle_sequence) / sizeof(uint16_t);

static volatile pwm_adv_duty_cycle_config m_pwm_adv_config = {
		.current_count = 0,
		.max_count = 32768,
		.duty_cycle_step = 0,
		.direction_up = true,
		.direction_mode_reverse = true
};

static void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{

}

void pwm_adv_init(void)
{
	uint32_t err_code;

	/* 2-channel PWM, 200Hz, output on LED pin. */
	app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, ADVERTISING_LED_PIN);

	/* Initialize and enable PWM. */
	err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm_ready_callback);
	APP_ERROR_CHECK(err_code);
	app_pwm_enable(&PWM1);
}

__STATIC_INLINE bool delay_duty_cycle_change()
{
	if (m_pwm_adv_config.current_count < m_pwm_adv_config.max_count)
	{
		m_pwm_adv_config.current_count++;
		return true;
	}
	else
	{
		m_pwm_adv_config.current_count = 0;
		return false;
	}
}

static uint8_t get_next_duty_cycle_step()
{
	uint8_t next_step = m_pwm_adv_config.direction_up ?
								m_pwm_adv_config.duty_cycle_step + 1 :
								m_pwm_adv_config.duty_cycle_step - 1;

	// at bounds limit change direction if in reverse mode
	if (m_pwm_adv_config.direction_mode_reverse && (next_step == 0 || next_step == sequence_size - 1))
	{
		m_pwm_adv_config.direction_up = !m_pwm_adv_config.direction_up;
	}
	// out of bounds reset back to beginning in non-reverse mode
	else if (next_step < 0 || next_step > sequence_size -1)
	{
		next_step = (next_step + sequence_size) % sequence_size;
	}

	m_pwm_adv_config.duty_cycle_step = next_step;

	return next_step;
}

void run_pwm_adv_indication()
{
	if (delay_duty_cycle_change()) {
		return;
	}

	uint16_t duty_cycle = duty_cycle_sequence[get_next_duty_cycle_step()];
	/* Set the duty cycle - keep trying until PWM is ready... */
	while (app_pwm_channel_duty_set(&PWM1, 0, duty_cycle) == NRF_ERROR_BUSY);
}

void set_pwm_adv_mode(pwm_adv_mode_t adv_mode)
{
	switch (adv_mode) {
		case PWM_ADV_INDICATE_DIRECTED_ADVERTISING:
		case PWM_ADV_INDICATE_FAST_ADVERTISING:
			m_pwm_adv_config.max_count = 16384;
			m_pwm_adv_config.direction_mode_reverse = true;
			break;
		case PWM_ADV_INDICATE_SLOW_ADVERTISING:
			m_pwm_adv_config.max_count = 32768;
			m_pwm_adv_config.direction_mode_reverse = true;
			break;
		case PWM_ADV_INDICATE_FAST_WHITELIST_ADVERTISING:
			m_pwm_adv_config.max_count = 16384;
			m_pwm_adv_config.direction_up = true;
			m_pwm_adv_config.direction_mode_reverse = false;
			break;
		case PWM_ADV_INDICATE_SLOW_WHITELIST_ADVERTISING:
			m_pwm_adv_config.max_count = 32768;
			m_pwm_adv_config.direction_up = false;
			m_pwm_adv_config.direction_mode_reverse = false;
			break;
		case PWM_ADV_INDICATE_IDLE:
			break;
		default:
			break;
	}
}
