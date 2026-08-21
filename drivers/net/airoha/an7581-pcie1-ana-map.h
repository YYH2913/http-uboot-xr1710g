/* SPDX-License-Identifier: GPL-2.0 */
/*
 * AN7581 Ethernet PHYA to PCIe1 lane1 field mapping.
 *
 * Generated from the one-to-one reg_field entries in Linux 6.18
 * drivers/net/pcs/airoha/pcs-an7581.c. Keep this table in source-register
 * order so callers can stop scanning once src_reg is greater than the
 * requested register.
 */

#ifndef __AN7581_PCIE1_ANA_MAP_H
#define __AN7581_PCIE1_ANA_MAP_H

struct an7581_pcie1_ana_field_map {
	u16 src_reg;
	u16 dst_reg;
	u8 src_lsb;
	u8 dst_lsb;
	u8 width;
};

static const struct an7581_pcie1_ana_field_map an7581_pcie1_ana_map[] = {
	{ 0x000, 0x000,  0,  0,  1 }, /* AN7581_PCS_CMN_EN */
	{ 0x004, 0x004,  8,  8,  1 }, /* AN7581_PCS_JCPLL_LPF_SHCK_EN */
	{ 0x004, 0x004, 16, 16,  6 }, /* AN7581_PCS_JCPLL_CHP_IBIAS */
	{ 0x004, 0x004, 24, 24,  6 }, /* AN7581_PCS_JCPLL_CHP_IOFST */
	{ 0x008, 0x008,  0,  0,  5 }, /* AN7581_PCS_JCPLL_LPF_BR */
	{ 0x008, 0x008,  8,  8,  5 }, /* AN7581_PCS_JCPLL_LPF_BC */
	{ 0x008, 0x008, 16, 16,  5 }, /* AN7581_PCS_JCPLL_LPF_BP */
	{ 0x008, 0x008, 24, 24,  5 }, /* AN7581_PCS_JCPLL_LPF_BWR */
	{ 0x00c, 0x00c,  0,  0,  5 }, /* AN7581_PCS_JCPLL_LPF_BWC */
	{ 0x00c, 0x00c,  8,  8,  1 }, /* AN7581_PCS_JCPLL_KBAND_OPTION */
	{ 0x00c, 0x00c, 16, 16,  8 }, /* AN7581_PCS_JCPLL_KBAND_CODE */
	{ 0x00c, 0x00c, 24, 24,  3 }, /* AN7581_PCS_JCPLL_KBAND_DIV */
	{ 0x010, 0x010,  0,  0,  2 }, /* AN7581_PCS_JCPLL_KBAND_KFC */
	{ 0x010, 0x010,  8,  8,  2 }, /* AN7581_PCS_JCPLL_KBAND_KF */
	{ 0x010, 0x010, 16, 16,  2 }, /* AN7581_PCS_JCPLL_KBAND_KS */
	{ 0x014, 0x014,  0,  0,  2 }, /* AN7581_PCS_JCPLL_MMD_PREDIV_MODE */
	{ 0x014, 0x014, 24, 24,  1 }, /* AN7581_PCS_JCPLL_POSTDIV_D5 */
	{ 0x01c, 0x01c,  0,  0,  3 }, /* AN7581_PCS_JCPLL_RST_DLY */
	{ 0x01c, 0x01c,  8,  8,  1 }, /* AN7581_PCS_JCPLL_PLL_RSTB */
	{ 0x01c, 0x01c, 16, 16,  1 }, /* AN7581_PCS_JCPLL_SDM_DI_EN */
	{ 0x01c, 0x01c, 24, 24,  2 }, /* AN7581_PCS_JCPLL_SDM_DI_LS */
	{ 0x020, 0x020,  0,  0,  1 }, /* AN7581_PCS_JCPLL_SDM_IFM */
	{ 0x020, 0x020,  8,  8,  2 }, /* AN7581_PCS_JCPLL_SDM_MODE */
	{ 0x020, 0x020, 16, 16,  2 }, /* AN7581_PCS_JCPLL_SDM_ORD */
	{ 0x020, 0x020, 24, 24,  1 }, /* AN7581_PCS_JCPLL_SDM_OUT */
	{ 0x024, 0x024,  0,  0,  1 }, /* AN7581_PCS_JCPLL_SDM_HREN */
	{ 0x024, 0x024,  8,  8,  1 }, /* AN7581_PCS_JCPLL_TCL_AMP_EN */
	{ 0x024, 0x024, 16, 16,  3 }, /* AN7581_PCS_JCPLL_TCL_AMP_GAIN */
	{ 0x024, 0x024, 24, 24,  5 }, /* AN7581_PCS_JCPLL_TCL_AMP_VREF */
	{ 0x028, 0x028, 16, 16,  1 }, /* AN7581_PCS_JCPLL_TCL_LPF_EN */
	{ 0x028, 0x028, 24, 24,  3 }, /* AN7581_PCS_JCPLL_TCL_LPF_BW */
	{ 0x02c, 0x02c,  0,  0,  2 }, /* AN7581_PCS_JCPLL_VCODIV */
	{ 0x02c, 0x02c,  8,  8,  2 }, /* AN7581_PCS_JCPLL_VCO_CFIX */
	{ 0x02c, 0x02c, 16, 16,  1 }, /* AN7581_PCS_JCPLL_VCO_HALFLSB_EN */
	{ 0x02c, 0x02c, 24, 24,  3 }, /* AN7581_PCS_JCPLL_VCO_SCAPWR */
	{ 0x030, 0x030,  0,  0,  3 }, /* AN7581_PCS_JCPLL_VCO_TCLVAR */
	{ 0x030, 0x030,  3,  8,  3 }, /* AN7581_PCS_JCPLL_VCO_VCOVAR_BIAS_H */
	{ 0x030, 0x030,  8, 16,  3 }, /* AN7581_PCS_JCPLL_VCO_VCOVAR_BIAS_L */
	{ 0x030, 0x038, 16,  0,  1 }, /* AN7581_PCS_JCPLL_SSC_EN */
	{ 0x030, 0x038, 17,  8,  1 }, /* AN7581_PCS_JCPLL_SSC_PHASE_INI */
	{ 0x034, 0x038,  0, 16,  1 }, /* AN7581_PCS_JCPLL_SSC_TRI_EN */
	{ 0x034, 0x03c,  8,  0, 16 }, /* AN7581_PCS_JCPLL_SSC_DELTA1 */
	{ 0x038, 0x03c,  0, 16, 16 }, /* AN7581_PCS_JCPLL_SSC_DELTA */
	{ 0x038, 0x040, 16,  0, 16 }, /* AN7581_PCS_JCPLL_SSC_PERIOD */
	{ 0x048, 0x04c,  8, 24,  8 }, /* AN7581_PCS_JCPLL_SPARE_L */
	{ 0x048, 0x050, 16,  0,  5 }, /* AN7581_PCS_JCPLL_TCL_KBAND_VREF */
	{ 0x050, 0x054,  0, 24,  6 }, /* AN7581_PCS_TXPLL_CHP_IBIAS */
	{ 0x050, 0x058,  8,  0,  6 }, /* AN7581_PCS_TXPLL_CHP_IOFST */
	{ 0x050, 0x058, 16,  8,  5 }, /* AN7581_PCS_TXPLL_LPF_BR */
	{ 0x050, 0x058, 24, 16,  5 }, /* AN7581_PCS_TXPLL_LPF_BC */
	{ 0x054, 0x058,  0, 24,  5 }, /* AN7581_PCS_TXPLL_LPF_BP */
	{ 0x054, 0x05c,  8,  0,  5 }, /* AN7581_PCS_TXPLL_LPF_BWR */
	{ 0x054, 0x05c, 16,  8,  5 }, /* AN7581_PCS_TXPLL_LPF_BWC */
	{ 0x054, 0x05c, 24, 16,  1 }, /* AN7581_PCS_TXPLL_KBAND_OPTION */
	{ 0x058, 0x05c,  0, 24,  8 }, /* AN7581_PCS_TXPLL_KBAND_CODE */
	{ 0x058, 0x060,  8,  0,  3 }, /* AN7581_PCS_TXPLL_KBAND_DIV */
	{ 0x058, 0x060, 16,  8,  2 }, /* AN7581_PCS_TXPLL_KBAND_KFC */
	{ 0x058, 0x060, 24, 16,  2 }, /* AN7581_PCS_TXPLL_KBAND_KF */
	{ 0x05c, 0x060,  0, 24,  2 }, /* AN7581_PCS_TXPLL_KBAND_KS */
	{ 0x05c, 0x064,  8,  0,  1 }, /* AN7581_PCS_TXPLL_POSTDIV_EN */
	{ 0x05c, 0x064, 16,  8,  2 }, /* AN7581_PCS_TXPLL_MMD_PREDIV_MODE */
	{ 0x064, 0x068,  0, 24,  1 }, /* AN7581_PCS_TXPLL_REFIN_INTERNAL */
	{ 0x064, 0x06c,  8,  0,  2 }, /* AN7581_PCS_TXPLL_REFIN_DIV */
	{ 0x064, 0x06c, 16,  8,  3 }, /* AN7581_PCS_TXPLL_RST_DLY */
	{ 0x064, 0x06c, 24, 16,  1 }, /* AN7581_PCS_TXPLL_PLL_RSTB */
	{ 0x068, 0x06c,  0, 24,  1 }, /* AN7581_PCS_TXPLL_SDM_DI_EN */
	{ 0x068, 0x070,  8,  0,  2 }, /* AN7581_PCS_TXPLL_SDM_DI_LS */
	{ 0x068, 0x070, 16,  8,  1 }, /* AN7581_PCS_TXPLL_SDM_IFM */
	{ 0x068, 0x070, 24, 16,  2 }, /* AN7581_PCS_TXPLL_SDM_MODE */
	{ 0x06c, 0x070,  0, 24,  2 }, /* AN7581_PCS_TXPLL_SDM_ORD */
	{ 0x06c, 0x074,  8,  0,  1 }, /* AN7581_PCS_TXPLL_SDM_OUT */
	{ 0x06c, 0x074, 16,  8,  1 }, /* AN7581_PCS_TXPLL_SDM_HREN */
	{ 0x06c, 0x078, 24, 24,  1 }, /* AN7581_PCS_TXPLL_TCL_AMP_EN */
	{ 0x070, 0x074,  0, 24,  3 }, /* AN7581_PCS_TXPLL_TCL_AMP_GAIN */
	{ 0x070, 0x078,  8,  0,  5 }, /* AN7581_PCS_TXPLL_TCL_AMP_VREF */
	{ 0x074, 0x074,  0,  0,  1 }, /* AN7581_PCS_TXPLL_TCL_LPF_EN */
	{ 0x074, 0x07c,  8,  0,  3 }, /* AN7581_PCS_TXPLL_TCL_LPF_BW */
	{ 0x074, 0x07c, 16,  8,  2 }, /* AN7581_PCS_TXPLL_VCODIV */
	{ 0x074, 0x07c, 24, 16,  2 }, /* AN7581_PCS_TXPLL_VCO_CFIX */
	{ 0x078, 0x07c,  0, 24,  1 }, /* AN7581_PCS_TXPLL_VCO_HALFLSB_EN */
	{ 0x078, 0x080,  8,  0,  3 }, /* AN7581_PCS_TXPLL_VCO_SCAPWR */
	{ 0x078, 0x080, 16,  8,  3 }, /* AN7581_PCS_TXPLL_VCO_TCLVAR */
	{ 0x078, 0x080, 24, 16,  3 }, /* AN7581_PCS_TXPLL_VCO_VCOVAR_BIAS_H */
	{ 0x078, 0x080, 27, 24,  3 }, /* AN7581_PCS_TXPLL_VCO_VCOVAR_BIAS_L */
	{ 0x07c, 0x084,  0,  0,  1 }, /* AN7581_PCS_TXPLL_SSC_EN */
	{ 0x07c, 0x084,  8,  8,  1 }, /* AN7581_PCS_TXPLL_SSC_PHASE_INI */
	{ 0x07c, 0x084, 16, 16,  1 }, /* AN7581_PCS_TXPLL_SSC_TRI_EN */
	{ 0x080, 0x088,  0,  0, 16 }, /* AN7581_PCS_TXPLL_SSC_DELTA */
	{ 0x080, 0x088, 16, 16, 16 }, /* AN7581_PCS_TXPLL_SSC_DELTA1 */
	{ 0x084, 0x08c,  0,  0, 16 }, /* AN7581_PCS_TXPLL_SSC_PERIOD */
	{ 0x084, 0x08c, 16, 16,  2 }, /* AN7581_PCS_TXPLL_LDO_OUT */
	{ 0x084, 0x08c, 24, 24,  2 }, /* AN7581_PCS_TXPLL_LDO_VCO_OUT */
	{ 0x094, 0x09c,  0,  0,  5 }, /* AN7581_PCS_TXPLL_TCL_KBAND_VREF */
	{ 0x0c4, 0x0e8,  0,  0,  1 }, /* AN7581_PCS_TX_CKLDO_EN */
	{ 0x0c4, 0x0e8, 24, 24,  1 }, /* AN7581_PCS_TX_DMEDGEGEN_EN */
	{ 0x0cc, 0x1ac, 16, 16,  1 }, /* AN7581_PCS_RX_PHY_CK_SEL */
	{ 0x0cc, 0x1ac, 24, 24,  1 }, /* AN7581_PCS_RX_PHY_CK_SEL_FORCE */
	{ 0x0d4, 0x1b4, 18, 18,  2 }, /* AN7581_PCS_RX_REV_1_SIGDET_ILEAK */
	{ 0x0d4, 0x1b4, 20, 20,  3 }, /* AN7581_PCS_RX_REV_1_FE_BUF2_BIAS_CTRL */
	{ 0x0d4, 0x1b4, 24, 24,  3 }, /* AN7581_PCS_RX_REV_1_FE_BUF1_BIAS_CTRL */
	{ 0x0d8, 0x1b8,  0,  0,  8 }, /* AN7581_PCS_RX_PHYCK_DIV */
	{ 0x0d8, 0x1b8,  8,  8,  2 }, /* AN7581_PCS_RX_PHYCK_SEL */
	{ 0x0d8, 0x1b8, 16, 16,  1 }, /* AN7581_PCS_RX_PHYCK_RSTB */
	{ 0x0d8, 0x1b8, 24, 24,  1 }, /* AN7581_PCS_RX_TDC_CK_SEL */
	{ 0x0dc, 0x1bc,  0,  0,  1 }, /* AN7581_PCS_CDR_PD_PICAL_CKD8_INV */
	{ 0x0dc, 0x1bc,  8,  8,  1 }, /* AN7581_PCS_CDR_PD_EDGE_DIS */
	{ 0x0e8, 0x1c8,  0,  0,  2 }, /* AN7581_PCS_CDR_LPF_RATIO */
	{ 0x0e8, 0x1c8,  8,  8, 19 }, /* AN7581_PCS_CDR_LPF_TOP_LIM */
	{ 0x0f4, 0x1d4, 24, 24,  1 }, /* AN7581_PCS_CDR_PR_INJ_FORCE_OFF */
	{ 0x0f8, 0x1d8,  0,  0,  7 }, /* AN7581_PCS_CDR_PR_BETA_DAC */
	{ 0x0f8, 0x1d8,  8,  8,  4 }, /* AN7581_PCS_CDR_PR_VCOADC_OS */
	{ 0x0f8, 0x1d8, 16, 16,  4 }, /* AN7581_PCS_CDR_PR_BETA_SEL */
	{ 0x0f8, 0x1d8, 24, 24,  3 }, /* AN7581_PCS_CDR_PR_KBAND_DIV */
	{ 0x0fc, 0x1dc,  0,  0,  3 }, /* AN7581_PCS_CDR_PR_VREG_IBAND_VAL */
	{ 0x0fc, 0x1dc,  8,  8,  3 }, /* AN7581_PCS_CDR_PR_VREG_CKBUF_VAL */
	{ 0x0fc, 0x1dc, 16, 16,  5 }, /* AN7581_PCS_CDR_PR_DAC_BAND */
	{ 0x0fc, 0x1dc, 24, 24,  2 }, /* AN7581_PCS_CDR_PR_FBKSEL */
	{ 0x10c, 0x1e8,  0, 24,  1 }, /* AN7581_PCS_CDR_PR_MONPR_EN */
	{ 0x10c, 0x1ec,  1,  0,  1 }, /* AN7581_PCS_CDR_PR_MONPI_EN */
	{ 0x10c, 0x1ec,  2,  8,  1 }, /* AN7581_PCS_CDR_PR_XFICK_EN */
	{ 0x10c, 0x1f0, 16,  0,  3 }, /* AN7581_PCS_CDR_BUF_IN_SR */
	{ 0x10c, 0x1f0, 19,  8,  1 }, /* AN7581_PCS_CDR_PR_CAP_EN */
	{ 0x10c, 0x1f0, 24, 16,  5 }, /* AN7581_PCS_RX_DAC_MON */
	{ 0x110, 0x1f4, 24, 24,  2 }, /* AN7581_PCS_RX_SIGDET_LPF_CTRL */
	{ 0x114, 0x1f8,  8,  8,  2 }, /* AN7581_PCS_RX_SIGDET_PEAK */
	{ 0x114, 0x1f8, 16, 16,  5 }, /* AN7581_PCS_RX_SIGDET_VTH_SEL */
	{ 0x118, 0x1fc,  0, 24,  1 }, /* AN7581_PCS_RX_FE_EQ_HZEN */
	{ 0x118, 0x200,  8,  0,  1 }, /* AN7581_PCS_RX_FE_VB_EQ1_EN */
	{ 0x118, 0x200, 16,  8,  1 }, /* AN7581_PCS_RX_FE_VB_EQ2_EN */
	{ 0x118, 0x200, 24, 16,  1 }, /* AN7581_PCS_RX_FE_VB_EQ3_EN */
	{ 0x11c, 0x200,  0, 24,  1 }, /* AN7581_PCS_RX_FE_VCM_GEN_PWDB */
	{ 0x120, 0x208,  8, 16, 10 }, /* AN7581_PCS_RX_OSCAL_FORCE */
	{ 0x13c, 0x228,  8, 16, 12 }, /* AN7581_PCS_AEQ_OFORCE */
	{ 0x144, 0x234, 24, 24,  1 }, /* AN7581_PCS_RX_DAC_D0_BYPASS_AEQ */
	{ 0x148, 0x238,  0,  0,  1 }, /* AN7581_PCS_RX_DAC_D1_BYPASS_AEQ */
	{ 0x148, 0x238,  8,  8,  1 }, /* AN7581_PCS_RX_DAC_E0_BYPASS_AEQ */
	{ 0x148, 0x238, 16, 16,  1 }, /* AN7581_PCS_RX_DAC_E1_BYPASS_AEQ */
	{ 0x148, 0x238, 24, 24,  1 }, /* AN7581_PCS_RX_DAC_EYE_BYPASS_AEQ */
};

#endif /* __AN7581_PCIE1_ANA_MAP_H */
