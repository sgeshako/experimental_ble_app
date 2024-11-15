#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include <string.h>
#include "spi.h"
#include "nrf_queue.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); /**< SPI instance. */
static volatile bool spi_xfer_done; /**< Flag used to indicate that SPI instance completed the transfer. */
static volatile bool m_set_mode_done = false; /**< Indicates if setting mode operation has ended. */
static bool is_spi_init = false;

static uint8_t m_tx_buf[2]; /**< TX buffer. */
static uint8_t m_rx_buf[15]; /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf); /**< Transfer length. */

static nrf_queue_t const * mp_queue = NULL;

/**
 * @brief SPI user event handler.
 * @param event
 */
static void spi_event_handler(nrf_drv_spi_evt_t const *p_event) {
	spi_xfer_done = true;
//	  NRF_LOG_INFO(" Sent. Address: %X\r\n", p_event->data.done.p_tx_buffer[0]);
//	  NRF_LOG_INFO(" Sent. Data: %X\r\n", p_event->data.done.p_tx_buffer[1]);
	if (!m_set_mode_done) {
		m_set_mode_done = true;
	}
    if (m_rx_buf[0] != 0) {

    	//			sample_combo_t new_sample;
    	//			memcpy(new_sample, &m_rx_buf[1], sizeof(new_sample));
    	//	    	if (m_sample_queue.p_cb->back == 0 || new_sample.acc.z.conv == 0)
    	//	    	{
    	//				NRF_LOG_RAW_INFO("Gyroscope: ");
    	//				NRF_LOG_RAW_INFO("[X: "NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(new_sample.gyro.x.conv * 0.00875f));
    	//				NRF_LOG_RAW_INFO("Y: "NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(new_sample.gyro.y.conv * 0.00875f));
    	//				NRF_LOG_RAW_INFO("Z: "NRF_LOG_FLOAT_MARKER"]\r\n", NRF_LOG_FLOAT(new_sample.gyro.z.conv * 0.00875f));
    	//				NRF_LOG_RAW_INFO("Accelerator: ");
    	//				NRF_LOG_RAW_INFO("[x: "NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(new_sample.acc.x.conv * 0.061f));
    	//				NRF_LOG_RAW_INFO("y: "NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(new_sample.acc.y.conv * 0.061f));
    	//				NRF_LOG_RAW_INFO("z: "NRF_LOG_FLOAT_MARKER"]\r\n", NRF_LOG_FLOAT(new_sample.acc.z.conv * 0.061f));
    	//				NRF_LOG_RAW_INFO("Temperature: "NRF_LOG_FLOAT_MARKER" C\r\n", NRF_LOG_FLOAT(25 + new_sample.temp.conv * 0.0625f));
    	//	    	}

		if (mp_queue != NULL) {
			nrf_queue_push(mp_queue, &m_rx_buf[1]);
		}
	}
}

/**
 * @brief Function for configuring LSM6DS33 accelerometer.
 */
static void LSM6DS33_configure(void) {
	uint8_t reg[4][2] = { { CTRL9_XL_ADDR, 0x38 }, // Enable accelerator x,y,z axes
			{ CTRL1_XL_ADDR, 0x60 }, // Acc. high-performance mode 416Hz
			{ CTRL10_C_ADDR, 0x38 }, // Enable gyroscope x,y,z axes
			{ CTRL2_G_ADDR, 0x60 }   // Gyro. high-performance mode 416Hz
	};

	for (int i = 0; i < sizeof(reg) / sizeof(reg[0]); i++) {
		m_set_mode_done = false;
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, reg[i], sizeof(reg[i]), NULL, 0));

		while (m_set_mode_done == false);
	}
}

/**
 * @brief SPI initialization.
 */
static void spi_init(void) {
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = SPI_SS_PIN;
	spi_config.miso_pin = SPI_MISO_PIN;
	spi_config.mosi_pin = SPI_MOSI_PIN;
	spi_config.sck_pin  = SPI_SCK_PIN;
	spi_config.mode = NRF_DRV_SPI_MODE_3;
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
	is_spi_init = true;
}

void read_sensor_blocking(sample_combo_t *p_new_sample, uint8_t sample_size) {
	if (!is_spi_init) {
		spi_init();
	}

	if (!m_set_mode_done) {
		LSM6DS33_configure();
	}

	mp_queue = NULL;
	// Reset rx buffer and transfer done flag
	memset(m_rx_buf, 0, sizeof(m_rx_buf));
	spi_xfer_done = false;

	m_tx_buf[0] = SPI_READ_REG(OUT_TEMP_L_REG);
	m_tx_buf[1] = 0;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, sizeof(m_rx_buf)));

	while (spi_xfer_done == false) {
		__WFE();
	}

//		*p_new_sample = *(sample_combo_t *)&m_rx_buf;
	memcpy(p_new_sample, &m_rx_buf[1], sample_size);
}

void read_sensor_non_blocking(void) {

	// Reset rx buffer and transfer done flag
	memset(m_rx_buf, 0, sizeof(m_rx_buf));
	spi_xfer_done = false;

	m_tx_buf[0] = SPI_READ_REG(OUT_TEMP_L_REG);
	m_tx_buf[1] = 0;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, sizeof(m_rx_buf)));
}

void poll_is_data_rdy(void) {

	// Reset rx buffer and transfer done flag
	memset(m_rx_buf, 0, sizeof(m_rx_buf));
	spi_xfer_done = false;

	m_tx_buf[0] = SPI_READ_REG(STATUS_REG);
	m_tx_buf[1] = 0;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, 2));
}

void spi_sensor_init(nrf_queue_t const * p_queue) {
	if (!is_spi_init) {
		spi_init();
	}

	if (!m_set_mode_done) {
		LSM6DS33_configure();
	}

	mp_queue = p_queue;
}
