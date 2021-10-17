//
// Created by jonas on 10/17/21.
//

#define DT_DRV_COMPAT nxp_flexio

#include "drivers/misc/flexio/flexio.h"

#include <device.h>
#include <logging/log.h>


LOG_MODULE_REGISTER(flexio, LOG_LEVEL_INF);


typedef struct flexio_data {
    // const struct device *dev;
} flexio_data;

static void flexio_isr(const struct device *dev) {
    LOG_INF("flexio isr for flexio @ %p", dev);
}

static int flexio_init(const struct device *dev) {
    const flexio_device_config *deviceConfig = dev->config;
    //LOG_INF("Flexio init, irq config func %p", deviceConfig->irq_config_func);
    //deviceConfig->irq_config_func(dev);
    flexio_config_t flexioConfig;
    FLEXIO_GetDefaultConfig(&flexioConfig);
    FLEXIO_Init(deviceConfig->flexioBase, &flexioConfig);
    FLEXIO_Enable(deviceConfig->flexioBase, true);
    return 0;
}

#define FLEXIO_DEVICE_INIT(n) \
    static void flexio_isr_config_func_##n(const struct device *dev); \
    static const flexio_device_config flexio_config_##n = { \
        .flexioBase= (FLEXIO_Type *) DT_INST_REG_ADDR(n),    \
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
        .clock_subsys = (clock_control_subsys_t) DT_INST_CLOCKS_CELL(n, name), \
        .irq_config_func = flexio_isr_config_func_##n, \
    }; \
    static flexio_data flexio_data_##n = {           \
                                  \
    }; \
    DEVICE_DT_INST_DEFINE(n, &flexio_init, NULL, \
                &flexio_data_##n, \
                &flexio_config_##n, POST_KERNEL, \
                CONFIG_SPI_INIT_PRIORITY, NULL); \
    static void flexio_isr_config_func_##n(const struct device *dev) {\
            LOG_INF("Configuring flexio isr");                  \
            IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), flexio_isr, DEVICE_DT_INST_GET(n), 0); \
            irq_enable(DT_INST_IRQN(n)); \
    }

DT_INST_FOREACH_STATUS_OKAY(FLEXIO_DEVICE_INIT)