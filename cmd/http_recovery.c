// SPDX-License-Identifier: GPL-2.0+

#include <command.h>
#include <env.h>
#include <linux/string.h>

int run_http_recovery(void);
int xg2010g_install_running_chainloader(void);

static int do_http_recovery(struct cmd_tbl *cmdtp, int flag, int argc,
			    char *const argv[])
{
	(void)cmdtp;
	(void)flag;
	(void)argc;
	(void)argv;

	env_set("ipaddr", "192.168.255.1");
	env_set("netmask", "255.255.255.0");
	env_set("gatewayip", "0.0.0.0");

	return run_http_recovery();
}

U_BOOT_CMD(
	http_recovery, 1, 0, do_http_recovery,
	"start the lwIP HTTP recovery server",
	""
);

static int do_chainloader_install(struct cmd_tbl *cmdtp, int flag, int argc,
				  char *const argv[])
{
	(void)cmdtp;
	(void)flag;

	if (argc != 2 || strcmp(argv[1], "XG2010G_INSTALL")) {
		printf("Refusing self-write without confirmation token XG2010G_INSTALL\n");
		return CMD_RET_USAGE;
	}

	return xg2010g_install_running_chainloader() ?
		CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	chainloader_install, 2, 0, do_chainloader_install,
	"install the running XG2010G chainloader FIT into its NAND slot",
	"XG2010G_INSTALL"
);
