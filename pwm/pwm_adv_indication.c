#include "pwm_adv_indication.h"
#include "app_pwm.h"
#include "app_error.h"
#include "boards.h"

#define ADVERTISING_LED_PIN             BSP_LED_1                             /**< Pin number for LED on board. */

// Create the instance "PWM1" using TIMER1.
APP_PWM_INSTANCE(PWM1,1);

/*
 * Pre-computed duty cycles after gamma correction
 */
static const uint16_t duty_cycle_sequence[] = { 0, 1, 2, 3, 5, 7, 10, 13, 17,
		22, 27, 33, 39, 46, 53, 61, 70, 79, 89, 100 };

static const uint8_t sequence_size = sizeof(duty_cycle_sequence) / sizeof(uint16_t);

static pwm_adv_duty_cycle_config pwm_adv_config = {
		.current_count = 0,
		.max_count = 32768,
		.duty_cycle_step = 0,
		.direction_up = true
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

void run_pwm_adv_indication()
{
	if (pwm_adv_config.current_count < pwm_adv_config.max_count)
	{
		pwm_adv_config.current_count++;
	}
	else
	{
		uint16_t duty_cycle = duty_cycle_sequence[pwm_adv_config.duty_cycle_step];
		/* Set the duty cycle - keep trying until PWM is ready... */
		while (app_pwm_channel_duty_set(&PWM1, 0, duty_cycle) == NRF_ERROR_BUSY);

		pwm_adv_config.current_count = 0;
		pwm_adv_config.duty_cycle_step = pwm_adv_config.direction_up ?
											pwm_adv_config.duty_cycle_step + 1 :
											pwm_adv_config.duty_cycle_step - 1;

		if (pwm_adv_config.duty_cycle_step == 0 || pwm_adv_config.duty_cycle_step == sequence_size - 1)
		{
			pwm_adv_config.direction_up = !pwm_adv_config.direction_up;
		}
	}
}
