// SPDX-License-Identifier: GPL-2.0
/*
 * Author: Christian Marangi <ansuelsmth@gmail.com>
 */

#include <asm/gpio.h>
#include <asm/global_data.h>
#include <dm/device.h>
#include <dm/ofnode.h>
#include <env.h>
#include <linux/kconfig.h>
#include <linux/string.h>

DECLARE_GLOBAL_DATA_PTR;

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = CFG_SYS_SDRAM_BASE + 0x100;

	return 0;
}

int run_http_recovery(void);

static int xr1710g_recovery_button_pressed(void)
{
	struct gpio_desc rec_gpio;
	ofnode root;
	int ret;

	if (!of_machine_is_compatible("econet,xr1710g"))
		return 0;

	memset(&rec_gpio, 0, sizeof(rec_gpio));
	root = ofnode_path("/");
	ret = gpio_request_by_name_nodev(root, "recovery-gpios", 0, &rec_gpio,
					 GPIOD_IS_IN);
	if (ret)
		return 0;

	ret = dm_gpio_get_value(&rec_gpio);
	dm_gpio_free(NULL, &rec_gpio);

	return ret > 0;
}

int board_late_init(void)
{
	if (!xr1710g_recovery_button_pressed())
		return 0;

	printf("Recovery button detected, starting web recovery...\n");
	env_set("ipaddr", "192.168.255.1");
	env_set("netmask", "255.255.255.0");
	env_set("gatewayip", "0.0.0.0");

	if (!env_get("recovery_addr"))
		env_set_hex("recovery_addr", CONFIG_SYS_LOAD_ADDR);

	if (IS_ENABLED(CONFIG_HTTPD_RECOVERY))
		run_http_recovery();
	else
		printf("HTTP recovery is not enabled.\n");

	return 0;
}
