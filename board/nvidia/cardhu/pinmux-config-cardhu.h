/*
 * Copyright (c) 2010-2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PINMUX_CONFIG_CARDHU_H_
#define _PINMUX_CONFIG_CARDHU_H_

#define DEFAULT_PINMUX(_pingroup, _mux, _pull, _tri, _io)	\
	{							\
		.pingroup	= PINGRP_##_pingroup,		\
		.func		= PMUX_FUNC_##_mux,		\
		.pull		= PMUX_PULL_##_pull,		\
		.tristate	= PMUX_TRI_##_tri,		\
		.io		= PMUX_PIN_##_io,		\
		.lock		= PMUX_PIN_LOCK_DEFAULT,	\
		.od		= PMUX_PIN_OD_DEFAULT,		\
		.ioreset	= PMUX_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pull, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= PINGRP_##_pingroup,		\
		.func		= PMUX_FUNC_##_mux,		\
		.pull		= PMUX_PULL_##_pull,		\
		.tristate	= PMUX_TRI_##_tri,		\
		.io		= PMUX_PIN_##_io,		\
		.lock		= PMUX_PIN_LOCK_##_lock,	\
		.od		= PMUX_PIN_OD_##_od,		\
		.ioreset	= PMUX_PIN_IO_RESET_DEFAULT,	\
	}

#define LV_PINMUX(_pingroup, _mux, _pull, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= PINGRP_##_pingroup,		\
		.func		= PMUX_FUNC_##_mux,		\
		.pull		= PMUX_PULL_##_pull,		\
		.tristate	= PMUX_TRI_##_tri,		\
		.io		= PMUX_PIN_##_io,		\
		.lock		= PMUX_PIN_LOCK_##_lock,	\
		.od		= PMUX_PIN_OD_DEFAULT,		\
		.ioreset	= PMUX_PIN_IO_RESET_##_ioreset	\
	}

static struct pingroup_config tegra3_pinmux_common[] = {
	/* SDMMC1 pinmux */
	DEFAULT_PINMUX(SDMMC1_CLK, SDMMC1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC1_CMD, SDMMC1, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT3, SDMMC1, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT2, SDMMC1, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT1, SDMMC1, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT0, SDMMC1, UP, NORMAL, INPUT),

	/* SDMMC3 pinmux */
	DEFAULT_PINMUX(SDMMC3_CLK, SDMMC3, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_CMD, SDMMC3, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT0, SDMMC3, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT1, SDMMC3, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT2, SDMMC3, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT3, SDMMC3, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT6, RSVD1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT7, RSVD1, NORMAL, NORMAL, INPUT),

	/* SDMMC4 pinmux */
	LV_PINMUX(SDMMC4_CLK, SDMMC4, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_CMD, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_DAT0, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_DAT1, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_DAT2, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_DAT3, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_DAT4, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_DAT5, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_DAT6, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_DAT7, SDMMC4, UP, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(SDMMC4_RST_N, RSVD1, DOWN, NORMAL, INPUT, DISABLE, DISABLE),

	/* I2C1 pinmux */
	I2C_PINMUX(GEN1_I2C_SCL, I2C1, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),
	I2C_PINMUX(GEN1_I2C_SDA, I2C1, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),

	/* I2C2 pinmux */
	I2C_PINMUX(GEN2_I2C_SCL, I2C2, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),
	I2C_PINMUX(GEN2_I2C_SDA, I2C2, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),

	/* I2C3 pinmux */
	I2C_PINMUX(CAM_I2C_SCL, I2C3, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),
	I2C_PINMUX(CAM_I2C_SDA, I2C3, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),

	/* I2C4 pinmux */
	I2C_PINMUX(DDC_SCL, I2C4, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),
	I2C_PINMUX(DDC_SDA, I2C4, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),

	/* Power I2C pinmux */
	I2C_PINMUX(PWR_I2C_SCL, I2CPWR, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),
	I2C_PINMUX(PWR_I2C_SDA, I2CPWR, NORMAL, NORMAL, INPUT, DISABLE, ENABLE),

	DEFAULT_PINMUX(ULPI_DATA0, UARTA, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA1, UARTA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(ULPI_DATA2, UARTA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(ULPI_DATA3, RSVD1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(ULPI_DATA4, UARTA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(ULPI_DATA5, UARTA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(ULPI_DATA6, UARTA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(ULPI_DATA7, UARTA, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(ULPI_CLK, UARTD, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(ULPI_DIR, UARTD, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(ULPI_NXT, UARTD, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(ULPI_STP, UARTD, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(DAP3_FS, I2S2, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP3_DIN, I2S2, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP3_DOUT, I2S2, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP3_SCLK, I2S2, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PV2, OWR, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(GPIO_PV3, RSVD1, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(CLK2_OUT, EXTPERIPH2, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(CLK2_REQ, DAP, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_PWR1, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_PWR2, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_SDIN, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_SDOUT, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_WR_N, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_CS0_N, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_DC0, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_SCK, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_PWR0, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_PCLK, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_DE, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_HSYNC, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_VSYNC, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D0, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D1, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D2, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D3, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D4, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D5, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D6, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D7, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D8, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D9, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D10, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D11, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D12, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D13, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D14, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D15, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D16, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D17, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D18, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D19, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D20, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D21, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D22, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_D23, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_CS1_N, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_M1, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(LCD_DC1, DISPA, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(CRT_HSYNC, CRT, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(CRT_VSYNC, CRT, NORMAL, NORMAL, OUTPUT),
	LV_PINMUX(VI_D0, RSVD1, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D1, SDMMC2, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D2, SDMMC2, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D3, SDMMC2, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D4, VI, NORMAL, NORMAL, OUTPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D5, SDMMC2, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D7, SDMMC2, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D10, RSVD1, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_MCLK, VI, UP, NORMAL, INPUT, DISABLE, DISABLE),
	DEFAULT_PINMUX(UART2_RXD, UARTB, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(UART2_TXD, UARTB, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(UART2_RTS_N, UARTB, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(UART2_CTS_N, UARTB, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(UART3_TXD, UARTC, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(UART3_RXD, UARTC, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(UART3_CTS_N, UARTC, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(UART3_RTS_N, UARTC, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(GPIO_PU0, RSVD1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PU1, RSVD1, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(GPIO_PU2, RSVD1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PU3, RSVD1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PU4, PWM1, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(GPIO_PU5, PWM2, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(GPIO_PU6, RSVD1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP4_FS, I2S3, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP4_DIN, I2S3, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP4_DOUT, I2S3, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP4_SCLK, I2S3, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(CLK3_OUT, EXTPERIPH3, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(CLK3_REQ, DEV3, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GMI_WP_N, GMI, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GMI_CS2_N, RSVD1, UP, NORMAL, INPUT), /* EN_VDD_BL1 */
	DEFAULT_PINMUX(GMI_AD8, PWM0, NORMAL, NORMAL, OUTPUT), /* LCD1_BL_PWM */
	DEFAULT_PINMUX(GMI_AD10, NAND, NORMAL, NORMAL, OUTPUT), /* LCD1_BL_EN */
	DEFAULT_PINMUX(GMI_A16, SPI4, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GMI_A17, SPI4, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GMI_A18, SPI4, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GMI_A19, SPI4, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(CAM_MCLK, VI_ALT2, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PCC1, RSVD1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PBB0, RSVD1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PBB3, VGP3, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PBB5, VGP5, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PBB6, VGP6, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PBB7, I2S4, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PCC2, I2S4, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(JTAG_RTCK, RTCK, NORMAL, NORMAL, OUTPUT),

	/* KBC keys */
	DEFAULT_PINMUX(KB_ROW0, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW1, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW2, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW3, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW4, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW5, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW6, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW7, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW8, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW9, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW10, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW11, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW12, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW13, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW14, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW15, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_COL0, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_COL1, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_COL2, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_COL3, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_COL4, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_COL5, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_COL6, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_COL7, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PV0, RSVD1, UP, NORMAL, INPUT),

	DEFAULT_PINMUX(CLK_32K_OUT, BLINK, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(SYS_CLK_REQ, SYSCLK, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(OWR, OWR, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP1_FS, I2S0, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP1_DIN, I2S0, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP1_DOUT, I2S0, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP1_SCLK, I2S0, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(CLK1_REQ, DAP, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(CLK1_OUT, EXTPERIPH1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(SPDIF_IN, SPDIF, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(SPDIF_OUT, SPDIF, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(DAP2_FS, I2S1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP2_DIN, I2S1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP2_DOUT, I2S1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(DAP2_SCLK, I2S1, NORMAL, NORMAL, INPUT),

	DEFAULT_PINMUX(SPI2_CS1_N, SPI2, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SPI1_MOSI, SPI1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(SPI1_SCK, SPI1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(SPI1_CS0_N, SPI1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(SPI1_MISO, SPI1, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(PEX_L0_PRSNT_N, PCIE, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(PEX_L0_RST_N, PCIE, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(PEX_L0_CLKREQ_N, PCIE, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(PEX_WAKE_N, PCIE, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(PEX_L1_PRSNT_N, PCIE, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(PEX_L1_RST_N, PCIE, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(PEX_L1_CLKREQ_N, PCIE, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(PEX_L2_PRSNT_N, PCIE, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(PEX_L2_RST_N, PCIE, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(PEX_L2_CLKREQ_N, PCIE, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(HDMI_CEC, CEC, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(HDMI_INT, RSVD1, NORMAL, TRISTATE, INPUT),

	/* GPIOs */
	/* SDMMC1 CD gpio */
	DEFAULT_PINMUX(GMI_IORDY, RSVD1, UP, NORMAL, INPUT),
	/* SDMMC1 WP gpio */
	LV_PINMUX(VI_D11, RSVD1, UP, NORMAL, INPUT, DISABLE, DISABLE),

	/* Touch panel GPIO */
	/* Touch IRQ */
	DEFAULT_PINMUX(GMI_AD12, NAND, UP, NORMAL, INPUT),

	/* Touch RESET */
	DEFAULT_PINMUX(GMI_AD14, NAND, NORMAL, NORMAL, OUTPUT),

	/* Power rails GPIO */
	DEFAULT_PINMUX(SPI2_SCK, GMI, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(GPIO_PBB4, VGP4, NORMAL, NORMAL, INPUT),
	DEFAULT_PINMUX(KB_ROW8, KBC, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT5, SDMMC3, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT4, SDMMC3, UP, NORMAL, INPUT),

	LV_PINMUX(VI_D6, VI, NORMAL, NORMAL, OUTPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D8, SDMMC2, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_D9, SDMMC2, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_PCLK, RSVD1, UP, TRISTATE, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_HSYNC, RSVD1, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
	LV_PINMUX(VI_VSYNC, RSVD1, NORMAL, NORMAL, INPUT, DISABLE, DISABLE),
};

static struct pingroup_config unused_pins_lowpower[] = {
	DEFAULT_PINMUX(GMI_WAIT, NAND, UP, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_ADV_N, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_CLK, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_CS3_N, NAND, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(GMI_CS7_N, NAND, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(GMI_AD0, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_AD1, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_AD2, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_AD3, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_AD4, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_AD5, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_AD6, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_AD7, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_AD9, PWM1, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(GMI_AD11, NAND, NORMAL, NORMAL, OUTPUT),
	DEFAULT_PINMUX(GMI_AD13, NAND, UP, NORMAL, INPUT),
	DEFAULT_PINMUX(GMI_WR_N, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_OE_N, NAND, NORMAL, TRISTATE, OUTPUT),
	DEFAULT_PINMUX(GMI_DQS, NAND, NORMAL, TRISTATE, OUTPUT),
};

#endif /* _PINMUX_CONFIG_CARDHU_H_ */