//
// Created by jonasotto on 9/15/22.
//

#define DT_DRV_COMPAT pixart_pmw3389

#include "pmw3389.h"

#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pmw3389, LOG_LEVEL_INF);

#define REG_Motion     0x02
#define BIT_Motion_MOT 7

#define REG_Delta_X_L	   0x03
#define REG_Delta_X_H	   0x04
#define REG_Delta_Y_L	   0x05
#define REG_Delta_Y_H	   0x06
#define REG_Resolution_L   0x0E
#define REG_Resolution_H   0x0F
#define REG_Power_Up_Reset 0x3A
#define REG_Motion_Burst   0x50

#define REG_Product_ID	    0x00
#define Expected_Product_ID 0x47

// TODO: Check timing interval vs delay

// Min command interval of write commands
#define T_SWW_US 180

// Min byte interval for read after write (..., data_write_1, address_read_2, ...)
#define T_SWR_US 180

// Min byte interval after read (..., data_read_1, address_2, ...)
#define T_SRW_US 20
#define T_SRR_US 20

// Min delay between address and data bytes for read
#define T_SRAD_US 160

// Min delay between address and data bytes for burst motion read
#define T_SRAD_MOTBR_US 35

/*
 * Chip SPI Interface description
 *
 * Whenever NCS goes high, the entire transaction is aborted, the serial port will be reset.
 * All transactions should be framed by NCS (do not leave NCS low if not used)
 * NCS must be raised after burst-mode transaction, or to terminate burst-mode.
 *
 * Chip reads MOSI on SCLK rising edge.
 * Chip writes MISO on SCLK falling edge.
 *
 * Write Operation:
 * Pull NCS low, delay 120ns
 * 1-bit, 7-bit address, 8-bit data.
 * (delay T_SCLK_NCS_WRITE (35us), pull NCS high)
 *
 * Read Operation:
 * 0-bit, 7-bit address, T_SRAD delay, 8-bit data
 * (delay T_SCLK_NCS_READ (120ns), pull NCS high)
 */

struct pmw3389_config {
	struct spi_dt_spec spi;
	int resolution_cpi; // [counts/inch]
};

struct pmw3389_data {
	int16_t delta_x;
	int16_t delta_y;
};

/**
 * @param reg 7-bit register address
 */
void write_register(const struct spi_dt_spec *spec, uint8_t reg, uint8_t data)
{
	// Set first address bit to 1 to indicate write
	uint8_t tx_data[] = {reg | 0b10000000, data};
	struct spi_buf tx_buffer = {.buf = tx_data, .len = sizeof(tx_data)};
	struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

	spi_write_dt(spec, &tx_buffer_set);
}

static void send_byte(const struct spi_dt_spec *spec, uint8_t data)
{
	uint8_t tx_data[] = {data};
	struct spi_buf tx_buffer = {.buf = tx_data, .len = sizeof(tx_data)};
	struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

	spi_write_dt(spec, &tx_buffer_set);
}

static uint8_t receive_byte(const struct spi_dt_spec *spec)
{
	uint8_t data;
	struct spi_buf rx_buffer = {.buf = &data, .len = 1};
	struct spi_buf_set rx_buffer_set = {.buffers = &rx_buffer, .count = 1};

	spi_read_dt(spec, &rx_buffer_set);
	return data;
}

void burst_read_motion(const struct device *dev, uint8_t burst_register, uint8_t *out)
{
	const struct pmw3389_config *config = dev->config;

	// Write any value to Motion_Burst register
	write_register(&config->spi, REG_Motion_Burst, 0);

	// Lower NCS
	// Send Motion_Burst address (0x50).
	send_byte(&config->spi, REG_Motion_Burst);

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

uint8_t read_register(const struct spi_dt_spec *spec, uint8_t addr)
{
	// Write address
	send_byte(spec, addr & ~(1 << 7));
	// Wait T_SRAD
	k_busy_wait(T_SRAD_US);
	// Read Data
	return receive_byte(spec);
}

/**
 * Read multiple registers
 */
void read_multiple(const struct spi_dt_spec *spec, const uint8_t addresses[], int nr_addresses,
		   uint8_t data_out[])
{
	for (int i = 0; i < nr_addresses; i++) {
		data_out[i] = read_register(spec, addresses[i]);
		// Wait T_SRR if not last
		if (i != nr_addresses - 1) {
			k_busy_wait(T_SRR_US);
		}
	}

	// T_SCLK_NCS_READ: omitted because small (120ns)
	spi_release_dt(spec);
}

int pmw3389_init(const struct device *dev)
{
	LOG_INF("Initializing PMW3389");
	const struct pmw3389_config *config = dev->config;

	const struct spi_dt_spec *spec = &config->spi;

	// SPI: Manual CS control, Zephyr does delay between pulling NCS low and start, but does not
	//  wait after transmission. NCS is not pulled high automatically, as configured

	// 1. Apply Power
	// 2. Drive NCS high, then low (done by driver)
	// 3. Write 0x5A to Power_Up_Reset register
	LOG_INF("Resetting device");
	write_register(spec, REG_Power_Up_Reset, 0x5A);
	spi_release_dt(spec);
	// 4. Wait for at least 50ms.
	k_sleep(K_MSEC(50));

	// 5. Read from registers 0x02, 0x03, 0x04, 0x05 and 0x06 one time regardless of the motion
	// pin state.
	uint8_t addresses[] = {0x02, 0x03, 0x04, 0x05, 0x06};
	uint8_t dummy_read[sizeof(addresses)];
	read_multiple(spec, addresses, sizeof(addresses), dummy_read);

	// (6. Perform SROM download [Refer to 7.1 SROM Download].)

	// 7. Write to register 0x3D with value 0x80.
	// Last op was read if SROM DL was not performed -> wait T_SRW
	k_busy_wait(T_SRW_US);
	write_register(spec, 0x3D, 0x80);

	// Wait such that the time between last address bit of the next read and last data bit is
	// > T_SWR: Assume clock frequency is at the maximum of 2MHz -> 4us for 8 address bits
	// The missing SRR here are waited in the first loop iteration below.
	k_busy_wait(T_SWR_US - 4);

	// 8. Read register 0x3D at 1ms interval until value 0xC0 is obtained or read up to
	//    55ms. This register read interval must be carried out at 1ms interval with timing
	//    tolerance of +/- 1%.
	while (true) {
		// Do not wait T_SRR since the 1ms is way longer anyway
		uint8_t result_3D = read_register(spec, 0x3D);
		LOG_INF("Got %#x", result_3D);
		if (result_3D == 0xC0) {
			break;
		}
		k_busy_wait(1000);
	}

	// 9. Write to register 0x3D with value 0x00.
	k_busy_wait(T_SRW_US);
	write_register(spec, 0x3D, 0x00);

	// 10. Write 0x20 to register 0x10
	// 0x10 is Config2, 0x20=0b00100000 enables the REST mode and sets CPI to addect both X and
	// Y
	k_busy_wait(T_SWW_US - 4);
	write_register(spec, 0x10, 0x20);

	// 11. Load configuration for other registers.

	// Set resolution
	LOG_INF("Configuring resolution: %dcpi", config->resolution_cpi);
	if (config->resolution_cpi < 50 || config->resolution_cpi > 16000) {
		LOG_ERR("Resolution of %d is out of range [%d, %d]", config->resolution_cpi, 50,
			16000);
		return -EINVAL;
	}
	if (config->resolution_cpi % 50 != 0) {
		LOG_WRN("Resolution of %d is not a multiple of 50cpi!", config->resolution_cpi);
	}
	uint16_t resolution = config->resolution_cpi / 50;
	k_busy_wait(T_SWW_US - 4);
	write_register(spec, REG_Resolution_H, resolution >> 8);
	k_busy_wait(T_SWW_US - 4);
	write_register(spec, REG_Resolution_L, resolution & 0xFF);

	// Verify communication
	LOG_INF("Verifying communication by reading product ID");
	k_busy_wait(T_SWR_US - 4);
	uint8_t received_product_id = read_register(spec, REG_Product_ID);
	if (received_product_id != Expected_Product_ID) {
		LOG_ERR("Read product ID %#x, but expected %#x, could not verify communication "
			"with sensor!",
			received_product_id, Expected_Product_ID);
		return -1; // TODO: Proper error code
	}
	LOG_INF("Verified product ID!");

	spi_release_dt(spec);
	LOG_INF("Done!");
	return 0;
}

static bool is_bit_set(uint8_t value, uint8_t index)
{
	return (value & (1 << index)) != 0;
}

static int16_t signed_16_from_parts(uint8_t low, uint8_t high)
{
	int res = low;
	res |= high << 8;
	return (int16_t)res;
}

int pmw3389_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	if (chan != SENSOR_CHAN_PMW3389_DISTANCE_X && chan != SENSOR_CHAN_PMW3389_DISTANCE_Y) {
		return -EINVAL;
	}

	const struct pmw3389_config *config = dev->config;
	const struct spi_dt_spec *spec = &config->spi;

	struct pmw3389_data *data = dev->data;

	// For first motion read, write any value to Motion register first (?)
	write_register(spec, REG_Motion, 0x00);
	k_busy_wait(T_SWR_US - 4);

	uint8_t motion = read_register(spec, REG_Motion);
	if (is_bit_set(motion, BIT_Motion_MOT)) {
		// Motion has occurred!
		uint8_t results[4] = {0};
		uint8_t addresses[4] = {REG_Delta_X_L, REG_Delta_X_H, REG_Delta_Y_L, REG_Delta_Y_H};
		k_busy_wait(T_SRR_US);
		read_multiple(spec, addresses, 4, results);
		data->delta_x = signed_16_from_parts(results[0], results[1]);
		data->delta_y = signed_16_from_parts(results[2], results[3]);
	} else {
		// No motion has occurred
		data->delta_x = 0;
		data->delta_y = 0;
	}

	return 0;
}

int pmw3389_channel_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	const struct pmw3389_config *config = dev->config;
	const struct pmw3389_data *data = dev->data;

	int16_t counts;

	switch ((int)chan) {
	case SENSOR_CHAN_PMW3389_DISTANCE_X:
		counts = data->delta_x;
		break;
	case SENSOR_CHAN_PMW3389_DISTANCE_Y:
		counts = data->delta_y;
		break;
	default:
		return -ENOTSUP;
	}

	int conversion_err = sensor_value_from_double(
		val, (double)counts / (39.3700787 * config->resolution_cpi));
	assert(conversion_err == 0);

	return 0;
}

static const struct sensor_driver_api pmw3389_api = {
	.sample_fetch = pmw3389_sample_fetch,
	.channel_get = pmw3389_channel_get,
};

#define PMW3389_INIT(n)                                                                            \
	static struct pmw3389_data pmw3389_data_##n;                                               \
	static const struct pmw3389_config pmw3389_config_##n = {                                  \
		.spi = SPI_DT_SPEC_INST_GET(n,                                                     \
					    SPI_OP_MODE_MASTER | SPI_WORD_SET(8U) |                \
						    SPI_HOLD_ON_CS | SPI_MODE_CPOL |               \
						    SPI_MODE_CPHA,                                 \
					    0U),                                                   \
		.resolution_cpi = DT_INST_PROP(n, resolution)};                                    \
	DEVICE_DT_INST_DEFINE(n, &pmw3389_init, NULL, &pmw3389_data_##n, &pmw3389_config_##n,      \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &pmw3389_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3389_INIT)
