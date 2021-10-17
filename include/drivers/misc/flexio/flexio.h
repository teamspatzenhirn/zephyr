//
// Created by jonas on 10/17/21.
//

#ifndef DRIVERS_MISC_FLEXIO_H
#define DRIVERS_MISC_FLEXIO_H

#include <fsl_flexio.h>
#include <drivers/clock_control.h>

typedef struct flexio_device_config_ {
    FLEXIO_Type *flexioBase;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;
    void (*irq_config_func)(const struct device *dev);
} flexio_device_config;

#endif //DRIVERS_MISC_FLEXIO_H
