/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for STM32L476G-DISCO board */
static const struct pin_config pinconf[] = {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay) && CONFIG_SERIAL
	{STM32_PIN_PB6,  STM32L4X_PINMUX_FUNC_PB6_USART1_TX},
	{STM32_PIN_PG10, STM32L4X_PINMUX_FUNC_PG10_USART1_RX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart2), okay) && CONFIG_SERIAL
	{STM32_PIN_PA2, STM32L4X_PINMUX_FUNC_PA2_USART2_TX},
	{STM32_PIN_PD6, STM32L4X_PINMUX_FUNC_PD6_USART2_RX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart1), okay) && CONFIG_SERIAL
	{STM32_PIN_PG7, STM32L4X_PINMUX_FUNC_PG7_LPUART1_TX},
	{STM32_PIN_PG8, STM32L4X_PINMUX_FUNC_PG8_LPUART1_RX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C
	{STM32_PIN_PB8, STM32L4X_PINMUX_FUNC_PB8_I2C1_SCL},
	{STM32_PIN_PB7, STM32L4X_PINMUX_FUNC_PB7_I2C1_SDA},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay) && CONFIG_SPI
	{STM32_PIN_PA5, STM32L4X_PINMUX_FUNC_PA5_SPI1_SCK},
	{STM32_PIN_PB4, STM32L4X_PINMUX_FUNC_PB4_SPI1_MISO},
	{STM32_PIN_PB5, STM32L4X_PINMUX_FUNC_PB5_SPI1_MOSI},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm2), okay) && CONFIG_PWM
	{STM32_PIN_PA0, STM32L4X_PINMUX_FUNC_PA0_PWM2_CH1},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(sdmmc1), okay) && \
	CONFIG_DISK_ACCESS_STM32_SDMMC
	{STM32_PIN_PC8, STM32L4X_PINMUX_FUNC_PC8_SDMMC1_D0},
	{STM32_PIN_PC9, STM32L4X_PINMUX_FUNC_PC9_SDMMC1_D1},
	{STM32_PIN_PC10, STM32L4X_PINMUX_FUNC_PC10_SDMMC1_D2},
	{STM32_PIN_PC11, STM32L4X_PINMUX_FUNC_PC11_SDMMC1_D3},
	{STM32_PIN_PC12, STM32L4X_PINMUX_FUNC_PC12_SDMMC1_CK},
	{STM32_PIN_PD2, STM32L4X_PINMUX_FUNC_PD2_SDMMC1_CMD},
#endif
};

static int pinmux_stm32_init(const struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
	 CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
