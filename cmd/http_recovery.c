// SPDX-License-Identifier: GPL-2.0+

#include <command.h>
#include <env.h>

int run_http_recovery(void);

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
