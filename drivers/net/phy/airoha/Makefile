# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_PHY_AIROHA_AN8801) += air_an8801.o
obj-$(CONFIG_PHY_AIROHA_EN8811) += air_en8811.o
obj-$(CONFIG_PHY_AIROHA_EN8811_BUILTIN_FW) += air_en8811_fw.o
obj-$(CONFIG_PHY_AIROHA_PHYLIB) += air_phy_lib.o

EN8811H_FW := $(srctree)/drivers/net/phy/airoha/en8811h-xg2010g.lzma
AFLAGS_air_en8811_fw.o := -DEN8811H_FW_FILE=\"$(EN8811H_FW)\"
$(obj)/air_en8811_fw.o: $(EN8811H_FW)
