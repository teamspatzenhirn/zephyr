//
// Created by jonasotto on 9/15/22.
//

#define DT_DRV_COMPAT pixart_pwm3389

#include "pwm3389.h"

#include <zephyr/drivers/spi.h>

#define REG_Power_Up_Reset 0x3A
#define REG_Motion_Burst   0x50

struct pwm3389_config {
	struct spi_dt_spec spi;
};

struct pwm3389_data {
};

/**
 * @param reg 7-bit register address
 */
void write_register(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct pwm3389_config *config = dev->config;

	// Set first address bit to 1 to indicate write
	uint8_t tx_data[] = {reg | 0b10000000, data};
	struct spi_buf tx_buffer = {.buf = tx_data, .len = sizeof(tx_data)};
	struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

	spi_write_dt(&config->spi, &tx_buffer_set);
}

void send_byte(const struct device *dev, uint8_t data)
{
	const struct pwm3389_config *config = dev->config;

	uint8_t tx_data[] = {data};
	struct spi_buf tx_buffer = {.buf = tx_data, .len = sizeof(tx_data)};
	struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

	spi_write_dt(&config->spi, &tx_buffer_set);
}

void burst_read_motion(const struct device *dev, uint8_t burst_register, uint8_t *out)
{
	const struct pwm3389_config *config = dev->config;

	// Write any value to Motion_Burst register
	write_register(dev, REG_Motion_Burst, 0);

	// Lower NCS
	// Send Motion_Burst address (0x50).
	send_byte(dev, REG_Motion_Burst);

	// Wait for t_SRAD_MOTBR
	k_busy_wait(35);

	// Start reading SPI data continuously up to 12bytes.
	uint8_t tx_data[12] = {0};
	struct spi_buf tx_buffer = {.buf = tx_data, .len = sizeof(tx_data)};
	struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

	struct spi_buf rx_buffers = {.buf = out, .len = 12};
	struct spi_buf_set rx_buffer_set = {.buffers = &rx_buffers, .count = 1};
	spi_transceive_dt(&config->spi, &tx_buffer_set, &rx_buffer_set);

	// Motion burst may be terminated by pulling NCS high for at least t_BEXIT
	spi_release_dt(&config->spi);
	// 500ns k_usleep();

	// TODO: t_SRAD etc
}

int pwm3389_init(const struct device *dev)
{
	// 3. Write 0x5A to Power_Up_Reset register
	write_register(dev, REG_Power_Up_Reset, 0x5A);
	// 4. Wait for at least 50ms.
	k_busy_wait(50000); // TODO: dont

	// 5. Read from registers 0x02, 0x03, 0x04, 0x05 and 0x06 one time regardless of the motion
	// pin state.

	// * Perform SROM download [Refer to 7.1 SROM Download].
	// * Write to register 0x3D with value 0x80.
	// * Read register 0x3D at 1ms interval until value 0xC0 is obtained or read up to 55ms.
	// This register read interval must be carried out at 1ms interval with timing tolerance of
	// +/- 1%.
	// * Write to register 0x3D with value 0x00.
	// * Write 0x20 to register 0x10
	// * Load configuration for other registers.
	return 0;
}

int pwm3389_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	return 0;
}

int pwm3389_channel_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	return 0;
}

static const struct sensor_driver_api pwm3389_api = {
	.sample_fetch = pwm3389_sample_fetch,
	.channel_get = pwm3389_channel_get,
};

#define PWM3389_INIT(n)                                                                            \
	static struct pwm3389_data pwm3389_data_##n;                                               \
	static const struct pwm3389_config pwm3389_config_##n = {                                  \
		.spi = SPI_DT_SPEC_INST_GET(                                                       \
			n, SPI_OP_MODE_MASTER | SPI_WORD_SET(8U) | SPI_HOLD_ON_CS, 0U),            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &pwm3389_init, NULL, &pwm3389_data_##n, &pwm3389_config_##n,      \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &pwm3389_api);

DT_INST_FOREACH_STATUS_OKAY(PWM3389_INIT)
