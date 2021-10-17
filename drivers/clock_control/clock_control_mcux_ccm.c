/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_ccm
#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <dt-bindings/clock/imx_ccm.h>
#include <fsl_clock.h>

//#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(clock_control, LOG_LEVEL_INF);

#ifdef CONFIG_SPI_MCUX_LPSPI
static const clock_name_t lpspi_clocks[] = {
	kCLOCK_Usb1PllPfd1Clk,
	kCLOCK_Usb1PllPfd0Clk,
	kCLOCK_SysPllClk,
	kCLOCK_SysPllPfd2Clk,
};
#endif

//#ifdef CONFIG_HAS_FLEXIO
// Same for flexio1 and flexio2
static const clock_name_t flexio_clocks[] = {
        kCLOCK_AudioPllClk, // PLL4 (Audio PLL)
        kCLOCK_Usb1PllPfd2Clk, // PLL3 (USB1 PLL) PFD2
        kCLOCK_VideoPllClk, // PLL5 (Video PLL)
        kCLOCK_Usb1PllClk, // really pll3_sw_clk, which is pll3 main clock or CCM_PLL3_BYP. But since the datasheet
                           // says "This bit should only be used for testing purposes.", and I can't make sense of
                           // CCM_PLL3_BYP, assume pll3 main clock...
};
//#endif

static int mcux_ccm_on(const struct device *dev,
			      clock_control_subsys_t sub_system)
{
	return 0;
}

static int mcux_ccm_off(const struct device *dev,
			       clock_control_subsys_t sub_system)
{
	return 0;
}

static int mcux_ccm_get_subsys_rate(const struct device *dev,
				    clock_control_subsys_t sub_system,
				    uint32_t *rate)
{
	uint32_t clock_name = (uint32_t) sub_system;

	switch (clock_name) {

#ifdef CONFIG_I2C_MCUX_LPI2C
	case IMX_CCM_LPI2C_CLK:
		if (CLOCK_GetMux(kCLOCK_Lpi2cMux) == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 8
				/ (CLOCK_GetDiv(kCLOCK_Lpi2cDiv) + 1);
		} else {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_Lpi2cDiv) + 1);
		}

		break;
#endif

#ifdef CONFIG_SPI_MCUX_LPSPI
	case IMX_CCM_LPSPI_CLK:
	{
		uint32_t lpspi_mux = CLOCK_GetMux(kCLOCK_LpspiMux);
		clock_name_t lpspi_clock = lpspi_clocks[lpspi_mux];

		*rate = CLOCK_GetFreq(lpspi_clock)
			/ (CLOCK_GetDiv(kCLOCK_LpspiDiv) + 1);
		break;
	}
#endif

#ifdef CONFIG_UART_MCUX_LPUART
	case IMX_CCM_LPUART_CLK:
		if (CLOCK_GetMux(kCLOCK_UartMux) == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6
				/ (CLOCK_GetDiv(kCLOCK_UartDiv) + 1);
		} else {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_UartDiv) + 1);
		}

		break;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usdhc1), okay) && CONFIG_DISK_DRIVER_SDMMC
	case IMX_CCM_USDHC1_CLK:
		*rate = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) /
				(CLOCK_GetDiv(kCLOCK_Usdhc1Div) + 1U);
		break;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usdhc2), okay) && CONFIG_DISK_DRIVER_SDMMC
	case IMX_CCM_USDHC2_CLK:
		*rate = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) /
				(CLOCK_GetDiv(kCLOCK_Usdhc2Div) + 1U);
		break;
#endif

#ifdef CONFIG_DMA_MCUX_EDMA
	case IMX_CCM_EDMA_CLK:
		*rate = CLOCK_GetIpgFreq();
		break;
#endif

#ifdef CONFIG_UART_MCUX_IUART
	case IMX_CCM_UART_CLK:
		*rate = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) /
				(CLOCK_GetRootPreDivider(kCLOCK_RootUart4)) /
				(CLOCK_GetRootPostDivider(kCLOCK_RootUart4)) /
				10;
		break;
#endif

#ifdef CONFIG_CAN_MCUX_FLEXCAN
	case IMX_CCM_CAN_CLK:
	{
		uint32_t can_mux = CLOCK_GetMux(kCLOCK_CanMux);

		if (can_mux == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 8
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		} else if  (can_mux == 1) {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		} else {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		}
	} break;
#endif

#ifdef CONFIG_COUNTER_MCUX_GPT
	case IMX_CCM_GPT_CLK:
		*rate = CLOCK_GetFreq(kCLOCK_PerClk);
		break;
#endif

//#ifdef CONFIG_SPI_MCUX_FLEXIO
        case IMX_CCM_FLEXIO1_CLK: {
            uint32_t flexio1_mux = CLOCK_GetMux(kCLOCK_Flexio1Mux);
            clock_name_t flexio1_clock = flexio_clocks[flexio1_mux];
            *rate = CLOCK_GetFreq(flexio1_clock)
                    / (CLOCK_GetDiv(kCLOCK_Flexio1PreDiv) + 1)
                    / (CLOCK_GetDiv(kCLOCK_Flexio1Div) + 1);
            break;
        }
        case IMX_CCM_FLEXIO2_CLK: {
            LOG_INF("flexio2/3 clock freq");
            uint32_t flexio2_mux = CLOCK_GetMux(kCLOCK_Flexio2Mux);
            clock_name_t flexio2_clock = flexio_clocks[flexio2_mux];
            *rate = CLOCK_GetFreq(flexio2_clock)
                    / (CLOCK_GetDiv(kCLOCK_Flexio2PreDiv) + 1)
                    / (CLOCK_GetDiv(kCLOCK_Flexio2Div) + 1);
            break;
        }
//#endif

    }

	return 0;
}

static int mcux_ccm_init(const struct device *dev)
{
	return 0;
}

static const struct clock_control_driver_api mcux_ccm_driver_api = {
	.on = mcux_ccm_on,
	.off = mcux_ccm_off,
	.get_rate = mcux_ccm_get_subsys_rate,
};

DEVICE_DT_INST_DEFINE(0,
		    &mcux_ccm_init,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &mcux_ccm_driver_api);
