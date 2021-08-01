#include <stdint.h>

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
 * @brief Function for bulk reading all sensor outputs
 */
void bulk_read(sample_combo_t * p_new_sample, uint8_t sample_size);


/**@brief SPI Read event type. */
typedef enum
{
    SPI_EVT_READ_STATUS,                            /**< Read status register event. */
    SPI_EVT_READ_SAMPLE,                            /**< Read sampled data event. */
} spi_read_evt_type_t;

/**@brief SPI Read event. */
typedef struct
{
    spi_read_evt_type_t evt_type;                                  /**< Type of event. */
} spi_read_evt_t;


typedef void (*gatts_value_update_handler_t) (uint8_t * p_sample, uint16_t sample_length);
typedef void (*on_sensor_read_handler_t) (volatile spi_read_evt_t * spi_read_evt, uint8_t * p_read, uint16_t length);

void configure(on_sensor_read_handler_t on_sensor_read_handler);

void bulk_read_into_gatts(void);

void poll_is_data_rdy(void);
