//
// Created by jonas on 10/17/21.
//

#include <device.h>
#include <drivers/led_strip.h>
#include <logging/log.h>
#include <drivers/misc/flexio/flexio.h>
#include <drivers/clock_control.h>


LOG_MODULE_REGISTER(ws2812_flexio, LOG_LEVEL_INF);
#define DT_DRV_COMPAT worldsemi_ws2812_flexio

typedef struct ws2812_flexio_config {
    const struct device *flexio;
    uint8_t timer;
    uint8_t shifter;
    uint8_t output_pin;
} ws2812_flexio_config;

typedef struct ws2812_flexio_data {

} ws2812_flexio_data;


static int ws2812_flexio_init(const struct device *dev) {
    const ws2812_flexio_config *config = dev->config;
    const flexio_device_config *busConf = config->flexio->config;

    flexio_shifter_config_t shifterConfig = {
            .timerSelect=config->timer,
            .timerPolarity=kFLEXIO_ShifterTimerPolarityOnNegitive,
            .pinConfig=kFLEXIO_PinConfigOutput,
            .pinSelect=config->output_pin,
            .pinPolarity=kFLEXIO_PinActiveHigh,
            .shifterMode=kFLEXIO_ShifterModeTransmit,
            .inputSource=kFLEXIO_ShifterInputFromPin,
            .shifterStop=kFLEXIO_ShifterStopBitDisable,
            .shifterStart=kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable
    };
    FLEXIO_SetShifterConfig(busConf->flexioBase, config->shifter, &shifterConfig);

    // Lower 8 bit configure baudrate: When lower 8 bit decrement to 0, timer output is toggled.
    uint32_t baudrate = 5000000; // 5MHz
    // TODO: Calculate and report effective frequency
    // TODO: Choose baudrate/pattern for specified timing
    uint32_t srcClock_Hz;
    if (clock_control_get_rate(busConf->clock_dev, busConf->clock_subsys, &srcClock_Hz)) {
        return -EINVAL;
    }
    LOG_INF("src clock rate %d", srcClock_Hz);
    uint16_t timerDiv = (uint16_t) (srcClock_Hz / baudrate);
    timerDiv = timerDiv / 2U - 1U;

    LOG_INF("timerdiv %d", timerDiv);

    uint16_t timerCmp = (8U * 2U - 1U) << 8U;
    timerCmp |= timerDiv;

    LOG_INF("timer cmp %#x", timerCmp);

    flexio_timer_config_t timerConfig = {
            .triggerSelect = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(config->shifter),
            .triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh,
            .triggerSource = kFLEXIO_TimerTriggerSourceInternal,
            .pinConfig = kFLEXIO_PinConfigOutputDisabled, // No SCK
            .timerMode = kFLEXIO_TimerModeDual8BitBaudBit,
            .timerOutput = kFLEXIO_TimerOutputZeroNotAffectedByReset,
            .timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput,
            .timerReset = kFLEXIO_TimerResetNever,
            .timerDisable = kFLEXIO_TimerDisableOnTimerCompare,
            .timerEnable = kFLEXIO_TimerEnableOnTriggerHigh,
            .timerStop = kFLEXIO_TimerStopBitEnableOnTimerCompare,
            .timerStart = kFLEXIO_TimerStartBitEnabled,
            .timerCompare = timerCmp,
    };
    FLEXIO_SetTimerConfig(busConf->flexioBase, config->timer, &timerConfig);

    FLEXIO_Enable(busConf->flexioBase, true);

    return 0;
}

int flexio_strip_update_rgb(const struct device *dev, struct led_rgb *pixels, size_t num_pixels) {
    const ws2812_flexio_config *config = dev->config;
    const flexio_device_config *busConf = config->flexio->config;
    printk("Enabling interrupts\n");
    FLEXIO_EnableShifterStatusInterrupts(busConf->flexioBase, 1UL << config->shifter);
    FLEXIO_EnableShifterErrorInterrupts(busConf->flexioBase, 1UL << config->shifter);

    printk("Loading SHIFTBUF\n");
    busConf->flexioBase->SHIFTBUF[config->shifter] = 0b00001111000011110000111100001111;
    busConf->flexioBase->SHIFTSTAT = 1;


    return 0;
}

int flexio_strip_update_channels(const struct device *dev, uint8_t *channels, size_t num_channels) {
    LOG_ERR("update_channels not implemented");
    return -ENOTSUP;
}

static const struct led_strip_driver_api ws2812_flexio_api = {
        .update_rgb = flexio_strip_update_rgb,
        .update_channels = flexio_strip_update_channels,
};

#define WS2812_FLEXIO_DEVICE_INIT(n) \
    static const ws2812_flexio_config ws2812_flexio_config_##n = { \
        .flexio = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(n))), \
        .timer = DT_INST_PROP(n, timer), \
        .shifter=DT_INST_PROP(n, shifter), \
        .output_pin = DT_INST_PROP(n, output_pin)\
    }; \
    static ws2812_flexio_data ws2812_flexio_data_##n = { \
    }; \
    DEVICE_DT_INST_DEFINE(n, &ws2812_flexio_init, NULL, \
                &ws2812_flexio_data_##n, \
                &ws2812_flexio_config_##n, \
                POST_KERNEL, \
                CONFIG_LED_STRIP_INIT_PRIORITY, \
                &ws2812_flexio_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_FLEXIO_DEVICE_INIT)