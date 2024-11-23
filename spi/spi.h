#include <stdint.h>
#include "nrf_queue.h"

/**
 * LSM6DS33 Register addresses
 */
#define WHO_AM_I_REG 0x0F
#define OUT_TEMP_L_REG 0x20
#define CTRL9_XL_ADDR 0x18
#define CTRL1_XL_ADDR 0x10
#define CTRL10_C_ADDR 0x19
#define CTRL2_G_ADDR 0x11
#define STATUS_REG 0x1E

#define SPI_READ_REG(reg) 0x80 | reg

/**
 * @brief Union of high and low 8-bit parts of a sample concatenated into a 16 bit.
 */
typedef union
{
    uint8_t  high_low[2];
    int16_t  conv;
} elem16_t;

/**
 * @brief Structure for holding samples from accelerometer/gyroscope.
 */
typedef struct
{
    elem16_t  x;
    elem16_t  y;
    elem16_t  z;
} sample16_t;

/**
 * @brief Structure for holding all the information read from accelerometer, gyroscope and temperature sensor.
 */
typedef struct
{
	  elem16_t   temp;
	  sample16_t gyro;
	  sample16_t acc;
} sample_combo_t;

/**
 * @brief Function for bulk reading all sensor outputs in blocking mode
 */
void read_sensor_blocking(sample_combo_t * p_new_sample, uint8_t sample_size);

/**
 * @brief Function for initializing SPI driver and LSM6DS33 sensor control registers.
 *
 * @param  p_queue   Pointer to queue that will store SPI reads in non-blocking mode.
 */
void spi_sensor_init(nrf_queue_t const * p_queue);

/**
 * @brief Function for bulk reading all sensor outputs, but return immediately (no blocking)
 */
void read_sensor_non_blocking(void);

void poll_is_data_rdy(void);
