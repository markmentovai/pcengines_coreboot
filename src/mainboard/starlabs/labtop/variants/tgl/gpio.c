/* SPDX-License-Identifier: GPL-2.0-only */

#include <variants.h>

/*
 * All definitions are taken from a comparison of the output of "inteltool -a"
 * using the stock BIOS and with coreboot.
 */

/* Early pad configuration in romstage. */
const struct pad_config early_gpio_table[] = {
	PAD_CFG_NF(GPP_C20, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_C21, NONE, DEEP, NF1),
};

const struct pad_config *variant_early_gpio_table(size_t *num)
{
	*num = ARRAY_SIZE(early_gpio_table);
	return early_gpio_table;
}

/* Pad configuration in ramstage.c */
const struct pad_config gpio_table[] = {
	PAD_CFG_NF(GPD0, UP_20K, PWROK, NF1),
	PAD_CFG_NF(GPD1, NATIVE, PWROK, NF1),
	PAD_CFG_NF(GPD2, NATIVE, PWROK, NF1),
	PAD_CFG_NF(GPD3, UP_20K, PWROK, NF1),
	PAD_CFG_NF(GPD4, NONE, PWROK, NF1),
	PAD_CFG_NF(GPD5, NONE, PWROK, NF1),
	PAD_CFG_NF(GPD6, NONE, PWROK, NF1),
	PAD_CFG_GPO(GPD7, 0, PWROK),
	PAD_CFG_NF(GPD8, NONE, PWROK, NF1),
	PAD_CFG_NF(GPD9, NONE, PWROK, NF1),
	PAD_CFG_NF(GPD10, NONE, PWROK, NF1),
	PAD_CFG_GPO(GPD11, 0, PWROK),
	PAD_CFG_NF(GPP_A0, UP_20K, DEEP, NF1),
	PAD_CFG_NF(GPP_A1, UP_20K, DEEP, NF1),
	PAD_CFG_NF(GPP_A2, UP_20K, DEEP, NF1),
	PAD_CFG_NF(GPP_A3, UP_20K, DEEP, NF1),
	PAD_CFG_NF(GPP_A4, UP_20K, DEEP, NF1),
	PAD_CFG_NF(GPP_A5, DN_20K, DEEP, NF1),
	PAD_CFG_NF(GPP_A6, NONE, DEEP, NF1),
	PAD_NC(GPP_A7, NONE),
	PAD_CFG_NF(GPP_A8, NONE, DEEP, NF2),
	PAD_CFG_NF(GPP_A9, NONE, DEEP, NF3),
	PAD_NC(GPP_A10, NONE),
	PAD_CFG_GPO(GPP_A11, 1, PLTRST),
	PAD_CFG_NF(GPP_A12, UP_20K, DEEP, NF1),
	PAD_NC(GPP_A13, NONE),
	PAD_CFG_GPO(GPP_A14, 0, PLTRST),
	PAD_NC(GPP_A15, NONE),
	PAD_CFG_NF(GPP_A16, NONE, DEEP, NF1),
	PAD_NC(GPP_A17, NONE),
	PAD_CFG_NF(GPP_A18, NONE, DEEP, NF1),
	PAD_NC(GPP_A19, NONE),
	PAD_NC(GPP_A20, NONE),
	PAD_NC(GPP_A21, NONE),
	PAD_NC(GPP_A22, NONE),
	PAD_CFG_GPO(GPP_A23, 0, PLTRST),
	PAD_CFG_NF(GPP_B0, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_B1, NONE, DEEP, NF1),
	PAD_NC(GPP_B2, NONE),
	PAD_NC(GPP_B3, NONE),
	PAD_NC(GPP_B4, NONE),
	PAD_NC(GPP_B5, NONE),
	PAD_NC(GPP_B6, NONE),
	PAD_NC(GPP_B7, NONE),
	PAD_NC(GPP_B8, NONE),
	PAD_CFG_NF(GPP_B9, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_B10, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_B11, NONE, PWROK, NF1),
	PAD_CFG_NF(GPP_B12, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_B13, NONE, DEEP, NF1),
	PAD_CFG_GPO(GPP_B14, 1, PLTRST),
	PAD_NC(GPP_B15, NONE),
	PAD_NC(GPP_B16, NONE),
	PAD_CFG_GPO(GPP_B17, 0, PWROK),
	PAD_CFG_GPO(GPP_B18, 0, DEEP),
	PAD_NC(GPP_B19, NONE),
	PAD_NC(GPP_B20, NONE),
	PAD_NC(GPP_B21, NONE),
	PAD_CFG_GPO(GPP_B22, 0, DEEP),
	PAD_CFG_GPO(GPP_B23, 0, DEEP),
	PAD_CFG_NF(GPP_C0, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_C1, NONE, DEEP, NF1),
	PAD_CFG_GPO(GPP_C2, 0, DEEP),
	PAD_CFG_NF(GPP_C3, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_C4, NONE, DEEP, NF1),
	PAD_CFG_GPO(GPP_C5, 0, DEEP),
	PAD_CFG_NF(GPP_C6, NONE, PWROK, NF1),
	PAD_CFG_NF(GPP_C7, NONE, PWROK, NF1),
	PAD_CFG_NF(GPP_C8, NONE, DEEP, NF1),
	PAD_NC(GPP_C9, NONE),
	PAD_CFG_GPO(GPP_C10, 0, PWROK),
	PAD_CFG_GPO(GPP_C11, 0, PWROK),
	PAD_CFG_NF(GPP_C12, NONE, DEEP, NF1),
	PAD_CFG_GPO(GPP_C13, 1, PLTRST),
	PAD_NC(GPP_C14, NONE),
	PAD_NC(GPP_C15, NONE),
	PAD_CFG_NF(GPP_C16, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_C17, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_C18, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_C19, NONE, DEEP, NF1),
	PAD_NC(GPP_C22, NONE),
	PAD_NC(GPP_C23, NONE),
	PAD_NC(GPP_D0, NONE),
	PAD_NC(GPP_D1, NONE),
	PAD_CFG_GPI(GPP_D2, NONE, DEEP),
	PAD_CFG_GPI(GPP_D3, NONE, DEEP),
	PAD_NC(GPP_D4, NONE),
	PAD_CFG_NF(GPP_D5, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_D6, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_D7, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_D8, NONE, DEEP, NF1),
	PAD_NC(GPP_D9, NATIVE),
	PAD_NC(GPP_D10, NATIVE),
	PAD_NC(GPP_D11, NATIVE),
	PAD_NC(GPP_D12, NATIVE),
	PAD_NC(GPP_D13, NONE),
	PAD_NC(GPP_D14, NONE),
	PAD_NC(GPP_D15, NONE),
	PAD_CFG_GPO(GPP_D16, 1, PLTRST),
	PAD_CFG_GPI(GPP_D17, NONE, DEEP),
	PAD_CFG_GPI(GPP_D18, NONE, DEEP),
	PAD_CFG_TERM_GPO(GPP_D19, 1, UP_20K, DEEP),
	PAD_NC(GPP_E0, NONE),
	PAD_NC(GPP_E1, NONE),
	PAD_NC(GPP_E2, NONE),
	PAD_CFG_NF(GPP_E3, NONE, DEEP, NF1),
	PAD_NC(GPP_E4, NONE),
	PAD_NC(GPP_E5, NONE),
	PAD_NC(GPP_E6, NONE),
	PAD_NC(GPP_E7, NONE),
	PAD_NC(GPP_E8, NONE),
	PAD_CFG_NF(GPP_E9, NONE, DEEP, NF1),
	PAD_NC(GPP_E10, NONE),
	PAD_NC(GPP_E11, NONE),
	PAD_NC(GPP_E12, NONE),
	PAD_NC(GPP_E13, NONE),
	PAD_CFG_NF(GPP_E14, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_E15, NONE, DEEP, NF2),
	PAD_CFG_NF(GPP_E16, NONE, DEEP, NF2),
	PAD_NC(GPP_E17, NONE),
	PAD_NC(GPP_E18, NATIVE),
	PAD_NC(GPP_E19, NATIVE),
	PAD_NC(GPP_E20, NATIVE),
	PAD_NC(GPP_E21, NATIVE),
	PAD_CFG_NF(GPP_E22, DN_20K, DEEP, NF2),
	PAD_CFG_GPO(GPP_E23, 0, DEEP),
	PAD_CFG_NF(GPP_F0, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_F1, UP_20K, DEEP, NF1),
	PAD_CFG_NF(GPP_F2, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_F3, UP_20K, DEEP, NF1),
	PAD_CFG_NF(GPP_F4, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_F5, NONE, DEEP, NF2),
	PAD_NC(GPP_F6, NONE),
	PAD_CFG_GPO(GPP_F7, 1, PLTRST),
	PAD_NC(GPP_F8, NONE),
	PAD_CFG_GPO(GPP_F9, 1, PLTRST),
	PAD_CFG_GPO(GPP_F10, 0, DEEP),
	PAD_NC(GPP_F11, NONE),
	PAD_NC(GPP_F12, NONE),
	PAD_NC(GPP_F13, NONE),
	PAD_NC(GPP_F14, NONE),
	PAD_NC(GPP_F15, NONE),
	PAD_NC(GPP_F16, NONE),
	PAD_NC(GPP_F17, NONE),
	PAD_NC(GPP_F18, NONE),
	PAD_NC(GPP_F19, NONE),
	PAD_CFG_NF(GPP_F20, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_F21, NONE, DEEP, NF1),
	PAD_NC(GPP_F22, NONE),
	PAD_NC(GPP_F23, NONE),
	PAD_CFG_GPO(GPP_H0, 0, DEEP),
	PAD_CFG_GPO(GPP_H1, 0, DEEP),
	PAD_CFG_GPO(GPP_H2, 0, DEEP),
	PAD_NC(GPP_H3, NONE),
	PAD_CFG_NF(GPP_H4, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_H5, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_H6, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_H7, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_H8, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_H9, NONE, DEEP, NF1),
	PAD_NC(GPP_H10, NONE),
	PAD_NC(GPP_H11, NONE),
	PAD_NC(GPP_H12, NONE),
	PAD_NC(GPP_H13, NONE),
	PAD_CFG_GPO(GPP_H14, 1, PLTRST),
	PAD_NC(GPP_H15, NONE),
	PAD_CFG_NF(GPP_H16, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_H17, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_H18, NONE, DEEP, NF1),
	PAD_NC(GPP_H19, NONE),
	PAD_NC(GPP_H20, NONE),
	PAD_NC(GPP_H21, NONE),
	PAD_NC(GPP_H22, NONE),
	PAD_CFG_GPO(GPP_H23, 0, DEEP),
	PAD_CFG_NF(GPP_R0, NONE, DEEP, NF1),
	PAD_CFG_NF(GPP_R1, NATIVE, DEEP, NF1),
	PAD_CFG_NF(GPP_R2, NATIVE, DEEP, NF1),
	PAD_CFG_NF(GPP_R3, NATIVE, DEEP, NF1),
	PAD_CFG_NF(GPP_R4, NONE, DEEP, NF1),
	PAD_CFG_GPO(GPP_R5, 0, PLTRST),
	PAD_CFG_GPO(GPP_R6, 1, PLTRST),
	PAD_NC(GPP_R7, NONE),
	PAD_NC(GPP_S0, NONE),
	PAD_NC(GPP_S1, NONE),
	PAD_NC(GPP_S2, NONE),
	PAD_NC(GPP_S3, NONE),
	PAD_NC(GPP_S4, NONE),
	PAD_NC(GPP_S5, NONE),
	PAD_NC(GPP_S6, NONE),
	PAD_NC(GPP_S7, NONE),
	PAD_CFG_NF(GPP_T2, DN_20K, DEEP, NF2),
	PAD_CFG_NF(GPP_T3, DN_20K, DEEP, NF2),
	PAD_NC(GPP_U4, NONE),
	PAD_NC(GPP_U5, NONE),
};

const struct pad_config *variant_gpio_table(size_t *num)
{
	*num = ARRAY_SIZE(gpio_table);
	return gpio_table;
}
