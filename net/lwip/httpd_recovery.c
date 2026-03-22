// SPDX-License-Identifier: GPL-2.0+
/*
 * Minimal HTTP upload recovery server using lwIP httpd
 * Serves a tiny upload page at / and handles POST to /upload/{firmware|uboot}
 */

#include <dm.h>
#include <env.h>
#include <log.h>
#include <malloc.h>
#include <command.h>
#include <mtd.h>
#include <net-lwip.h>
#include <net.h>
#include <initcall.h>
#include <ubi_uboot.h>
#include <watchdog.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <console.h>
#include <asm/global_data.h>

#include <lwip/netif.h>
#include <lwip/pbuf.h>
#include <lwip/timeouts.h>
#include <lwip/apps/httpd.h>
#include <lwip/apps/fs.h>
#include <version.h>
#include <timestamp.h>
#include <limits.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Upload buffer
 * Use env 'recovery_addr' if set, otherwise fall back to U-Boot 'loadaddr'.
 * Avoid hard-coding a RAM address which may overlap U-Boot/lwIP memory.
 */
/* Default maximum upload size in bytes (override with env 'recovery_max') */
#define RECOVERY_UPLOAD_MAX    (32 * 1024 * 1024UL)

/* Delay before reboot after flashing completes, to let browser finish reads */
#define REBOOT_DELAY_MS        3000

static u8 *recv_base;
static u32 recv_off;
static u32 recv_total;
static int post_ok;
static int upload_done;
static int flash_request;
static volatile int reboot_request;
/* Progress for /status polling */
static volatile u32 prog_total; /* combined total for backward compat */
static volatile u32 prog_done;  /* combined done for backward compat */
static volatile u32 prog_erase_total;
static volatile u32 prog_erase_done;
static volatile u32 prog_write_total;
static volatile u32 prog_write_done;
static volatile int prog_phase; /* 0 idle, 1 erase, 2 write, 3 done, -1 error */

static void post_delay_cb(void *arg)
{
    (void)arg;
    flash_request = 1;
}

static void reboot_delay_cb(void *arg)
{
    (void)arg;
    reboot_request = 1;
}

enum upload_target {
	TARGET_FIRMWARE = 0,
	TARGET_UBOOT,
};
static enum upload_target current_target = TARGET_FIRMWARE;

enum recovery_backend {
	RECOVERY_BACKEND_MTD = 0,
	RECOVERY_BACKEND_UBI,
};

struct recovery_target {
	enum recovery_backend backend;
	const char *name;
	const char *ubi_part;
	struct mtd_info *mtd;
	loff_t ofs;
	unsigned long long limit;
};

static const char *recovery_default_target(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "fit";
	case TARGET_UBOOT:
		return "uboot";
	}

	return "fit";
}

static const char *recovery_target_env(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "recovery_mtd";
	case TARGET_UBOOT:
		return "recovery_mtd_uboot";
	}

	return "recovery_mtd";
}

static const char *recovery_raw_env(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "recovery_dev";
	case TARGET_UBOOT:
		return "recovery_dev_uboot";
	}

	return "recovery_dev";
}

static const char *recovery_size_env(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "recovery_size";
	case TARGET_UBOOT:
		return "recovery_size_uboot";
	}

	return "recovery_size";
}

static ulong recovery_raw_offset(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return env_get_hex("recovery_ofs", 0x050000);
	case TARGET_UBOOT:
		return env_get_hex("uboot_ofs", 0x0);
	}

	return 0;
}

static const char *recovery_ubi_part(enum upload_target tgt)
{
	const char *part;

	switch (tgt) {
	case TARGET_UBOOT:
		part = env_get("recovery_ubi_part_uboot");
		break;
	case TARGET_FIRMWARE:
	default:
		part = env_get("recovery_ubi_part");
		break;
	}

	if (!part)
		part = env_get("recovery_ubi_part");

	return part ?: "ubi";
}

static int recovery_try_ubi_target(enum upload_target tgt,
				       struct recovery_target *target)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_volume_desc *desc;
	const char *volume = env_get(recovery_target_env(tgt));
	const char *part = recovery_ubi_part(tgt);

	if (!volume)
		volume = recovery_default_target(tgt);

	if (ubi_part((char *)part, NULL))
		return -ENODEV;

	desc = ubi_open_volume_nm(0, volume, UBI_READWRITE);
	if (IS_ERR_OR_NULL(desc))
		return -ENODEV;

	target->backend = RECOVERY_BACKEND_UBI;
	target->name = volume;
	target->ubi_part = part;
	target->limit = (unsigned long long)desc->vol->reserved_pebs *
			(unsigned long long)desc->vol->usable_leb_size;

	ubi_close_volume(desc);
	return 0;
#else
	return -ENODEV;
#endif
}

static int recovery_resolve_target(enum upload_target tgt,
				   struct recovery_target *target)
{
	const char *name = env_get(recovery_target_env(tgt));
	const char *raw;
	struct mtd_info *mtd;
	ulong ofs;

	memset(target, 0, sizeof(*target));

	if (!name)
		name = recovery_default_target(tgt);

	mtd_probe_devices();

	raw = env_get(recovery_raw_env(tgt));
	if (raw && *raw) {
		mtd = get_mtd_device_nm(raw);
		if (!IS_ERR_OR_NULL(mtd)) {
			ulong size_cap = env_get_hex(recovery_size_env(tgt), 0);

			ofs = recovery_raw_offset(tgt);
			target->backend = RECOVERY_BACKEND_MTD;
			target->name = raw;
			target->mtd = mtd;
			target->ofs = ofs;
			target->limit = mtd->size;
			if (ofs && target->limit > ofs)
				target->limit -= ofs;
			if (size_cap && target->limit > size_cap)
				target->limit = size_cap;
			return 0;
		}
	}

	mtd = get_mtd_device_nm(name);
	if (!IS_ERR_OR_NULL(mtd)) {
		target->backend = RECOVERY_BACKEND_MTD;
		target->name = name;
		target->mtd = mtd;
		target->limit = mtd->size;
		return 0;
	}

	if (!recovery_try_ubi_target(tgt, target))
		return 0;

	if (!raw) {
		switch (tgt) {
		case TARGET_FIRMWARE:
			raw = "nor0";
			break;
		case TARGET_UBOOT:
			raw = env_get("recovery_dev");
			if (!raw)
				raw = "nor0";
			break;
		}
	}

	mtd = get_mtd_device_nm(raw);
	if (IS_ERR_OR_NULL(mtd))
		return -ENODEV;

	ofs = recovery_raw_offset(tgt);
	target->backend = RECOVERY_BACKEND_MTD;
	target->name = raw;
	target->mtd = mtd;
	target->ofs = ofs;
	target->limit = mtd->size;
	if (ofs && target->limit > ofs)
		target->limit -= ofs;
	{
		ulong size_cap = env_get_hex(recovery_size_env(tgt), 0);

		if (size_cap && target->limit > size_cap)
			target->limit = size_cap;
	}

	return 0;
}

static void recovery_release_target(struct recovery_target *target)
{
	if (target->backend == RECOVERY_BACKEND_MTD && target->mtd)
		put_mtd_device(target->mtd);
}

/* Determine maximum payload size based on selected target and DTS-defined MTD
 * layout. Returns 0 on error. */
static unsigned long recovery_calc_target_max(enum upload_target tgt, loff_t *p_ofs)
{
	struct recovery_target target;
	unsigned long limit = 0;

	if (recovery_resolve_target(tgt, &target))
		return 0;

	if (p_ofs)
		*p_ofs = target.ofs;

	limit = (target.limit > ULONG_MAX) ? ULONG_MAX : (unsigned long)target.limit;
	recovery_release_target(&target);

	return limit;
}

/* Only dynamic endpoints here; static files come from fsdata */

/* lwIP httpd custom file hooks: serve only dynamic endpoints; static files via fsdata */
int fs_open_custom(struct fs_file *file, const char *name)
{
    const char *p;
    if (!file || !name)
        return 0;

    /* Normalize leading slash to make httpd default filenames work */
    p = name;
    if (*p == '/')
        p++;


    if (!strcmp(p, "status")) {
        static char page_status[512];
        char json[256];
        int ok = (prog_phase == 3);
        int err = (prog_phase == -1);
        int json_len = snprintf(json, sizeof(json),
                                "{\"in_progress\":%d,\"done\":%u,\"total\":%u,\"erase_done\":%u,\"erase_total\":%u,\"write_done\":%u,\"write_total\":%u,\"ok\":%d,\"error\":%d,\"phase\":%d}\n",
                                prog_phase > 0 && prog_phase < 3,
                                (unsigned)prog_done, (unsigned)prog_total,
                                (unsigned)prog_erase_done, (unsigned)prog_erase_total,
                                (unsigned)prog_write_done, (unsigned)prog_write_total,
                                ok, err, prog_phase);
        if (json_len < 0)
            return 0;
        if (json_len >= (int)sizeof(json))
            json_len = (int)sizeof(json) - 1;

        /* Build header with Content-Length for robustness */
        int hdr_len = snprintf(page_status, sizeof(page_status),
                               "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: %d\r\nConnection: close\r\n\r\n",
                               json_len);
        if (hdr_len < 0)
            return 0;
        if ((size_t)hdr_len >= sizeof(page_status))
            hdr_len = sizeof(page_status) - 1;
        /* Append body */
        size_t space = sizeof(page_status) - hdr_len - 1;
        size_t body_len = (json_len > (int)space) ? space : (size_t)json_len;
        memcpy(page_status + hdr_len, json, body_len);
        page_status[hdr_len + body_len] = '\0';

        file->data = page_status;
        file->len = hdr_len + body_len; /* actual header + body written */
        file->index = 0;
        file->flags = FS_FILE_FLAGS_HEADER_INCLUDED; /* header already included */
        return 1;
    }
    else if (!strcmp(p, "about")) {
        static char page_about[384];
        char json[256];
        int json_len =
#ifdef U_BOOT_DATE
            snprintf(json, sizeof(json),
                     "{\"u_boot\":\"%s (%s - %s %s)\"}\n",
                     U_BOOT_VERSION, U_BOOT_DATE, U_BOOT_TIME, U_BOOT_TZ);
#else
            snprintf(json, sizeof(json),
                     "{\"u_boot\":\"%s\"}\n",
                     U_BOOT_VERSION);
#endif
        if (json_len < 0)
            return 0;
        if (json_len >= (int)sizeof(json))
            json_len = (int)sizeof(json) - 1;

        int hdr_len = snprintf(page_about, sizeof(page_about),
                               "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: %d\r\nConnection: close\r\n\r\n",
                               json_len);
        if (hdr_len < 0)
            return 0;
        if ((size_t)hdr_len >= sizeof(page_about))
            hdr_len = sizeof(page_about) - 1;
        size_t space = sizeof(page_about) - hdr_len - 1;
        size_t body_len = (json_len > (int)space) ? space : (size_t)json_len;
        memcpy(page_about + hdr_len, json, body_len);
        page_about[hdr_len + body_len] = '\0';

        file->data = page_about;
        file->len = hdr_len + body_len;
        file->index = 0;
        file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
        return 1;
    }
    /* Do not intercept favicon/index/ok/fail: served by fsdata */
    /* let fsdata handle others */
    return 0;
}

void fs_close_custom(struct fs_file *file)
{
    (void)file;
}

int fs_read_custom(struct fs_file *file, char *buffer, int count)
{
    u32_t left;
    if (!file || !buffer || count <= 0)
        return FS_READ_EOF;
    left = file->len - file->index;
    if (left <= 0)
        return FS_READ_EOF;
    if ((u32_t)count > left)
        count = left;
    memcpy(buffer, file->data + file->index, count);
    file->index += count;
    return count;
}

/* bytes_left for custom files is handled by fs_read_custom + file->index; */

/* HTTP POST handlers */
err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd)
{
    (void)http_request; (void)http_request_len; (void)connection;
    /* Use automatic window management and keep the request path simple. */
    if (post_auto_wnd)
        *post_auto_wnd = 1;

    post_ok = 0;
    upload_done = 0;
    recv_off = 0;
    recv_total = 0;
    /* Reset progress state. */
    prog_phase = 0;
    prog_done = 0;
    prog_total = 0;
    prog_erase_done = 0;
    prog_erase_total = 0;
    prog_write_done = 0;
    prog_write_total = 0;

	    /* Accept optional query parameters after the target path. */
    if (!strncmp(uri, "/upload/firmware", 16) && (uri[16] == '\0' || uri[16] == '?'))
        current_target = TARGET_FIRMWARE;
    else if (!strncmp(uri, "/upload/uboot", 13) && (uri[13] == '\0' || uri[13] == '?'))
        current_target = TARGET_UBOOT;
    else if (!strncmp(uri, "/upload", 7) && (uri[7] == '\0' || uri[7] == '?'))
        current_target = TARGET_FIRMWARE;
    else {
        strlcpy(response_uri, "/fail.html", response_uri_len);
        return ERR_ARG;
    }

    {
        ulong env_max = env_get_hex("recovery_max", 0);
        loff_t tmpofs = 0;
        ulong dts_max = recovery_calc_target_max(current_target, &tmpofs);
        ulong max = dts_max ? dts_max : RECOVERY_UPLOAD_MAX;
        if (env_max && env_max < max)
            max = env_max; /* allow env to further cap */
        if (content_len <= 0 || (ulong)content_len > max) {
            printf("httpd: content_len %d exceeds allowed max %lu (target limit %lu, ofs 0x%llx)\n",
                   content_len, max, dts_max, (unsigned long long)tmpofs);
            strlcpy(response_uri, "/fail.html", response_uri_len);
            return ERR_ARG;
        }
    }

    recv_total = content_len;

    /*
     * Pick a stable upload buffer:
     * recovery_addr -> loadaddr -> CONFIG_SYS_LOAD_ADDR -> RAM fallback.
     */
    {
        ulong ram_start = (ulong)gd->ram_base;
        ulong ram_end = (ulong)gd->ram_base + (ulong)gd->ram_size;
        ulong base = env_get_hex("recovery_addr", 0);
        if (!base)
            base = env_get_hex("loadaddr", 0);
        if (!base)
            base = CONFIG_SYS_LOAD_ADDR;

        /* Keep the buffer inside usable RAM. */
        if (base < ram_start || (base + recv_total) > ram_end) {
            ulong fallback = ram_start + 0x01000000UL;
            if ((fallback >= ram_start) && ((fallback + recv_total) <= ram_end))
                base = fallback;
            else if ((ram_end - recv_total) > ram_start)
                base = ram_end - recv_total;
            else {
                printf("httpd: no sufficient RAM for upload (%u bytes)\n", recv_total);
                strlcpy(response_uri, "/fail.html", response_uri_len);
                return ERR_MEM;
            }
        }
        recv_base = (u8 *)base;
    }

    post_ok = 1;
    /* Leave response_uri untouched here so the POST can complete normally. */
    const char *tname = current_target == TARGET_FIRMWARE ? "firmware" : "uboot";
    printf("httpd: accepting upload of %u bytes for %s to 0x%08lx\n",
           recv_total, tname, (ulong)recv_base);
    return ERR_OK;
}

err_t httpd_post_receive_data(void *connection, struct pbuf *p)
{
	struct pbuf *q;

	if (!post_ok) {
		pbuf_free(p);
		return ERR_ARG;
	}

	/* Copy request payload into RAM and defer flash work to the main loop. */
	for (q = p; q != NULL; q = q->next) {
        size_t avail = recv_total - recv_off;
        size_t clen = q->len;
        if (clen > avail)
            clen = avail;
        memcpy(recv_base + recv_off, q->payload, clen);
        recv_off += clen;
    }
    pbuf_free(p);

    if (recv_off >= recv_total) {
        upload_done = 1;
    }

    return ERR_OK;
}

void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
    (void)connection;
    printf("httpd: post finished, %u/%u bytes received\n", recv_off, recv_total);
    /* Tell httpd which page to return after POST (keep user on main page) */
    if (post_ok && recv_total && (recv_off >= recv_total))
        strlcpy(response_uri, "/index.html", response_uri_len);
    else
        strlcpy(response_uri, "/fail.html", response_uri_len);

    /*
     * Delay flashing slightly so the browser can finish receiving the POST
     * response before erase/write work blocks the network loop.
     */
    if (post_ok && recv_total && (recv_off >= recv_total))
        sys_timeout(1000, post_delay_cb, NULL);
}

static int flash_image(void)
{
	struct recovery_target target;
	size_t retlen;
	int ret;

	if (!recv_off) {
		printf("No data received to flash\n");
		prog_phase = -1;
		return -EINVAL;
	}

	ret = recovery_resolve_target(current_target, &target);
	if (ret) {
		printf("No flash target found for upload type %d\n", current_target);
		prog_phase = -1;
		return ret;
	}

	if (recv_off > target.limit) {
		printf("Image size %u exceeds target size %llu\n",
		       recv_off, target.limit);
		recovery_release_target(&target);
		prog_phase = -1;
		return -EFBIG;
	}

	if (target.backend == RECOVERY_BACKEND_UBI) {
		printf("Writing %u bytes to UBI volume '%s' on '%s'...\n",
		       recv_off, target.name, target.ubi_part);
		prog_phase = 2;
		prog_done = 0;
		prog_total = recv_off;
		prog_erase_done = 0;
		prog_erase_total = 0;
		prog_write_done = 0;
		prog_write_total = recv_off;

		if (ubi_part((char *)target.ubi_part, NULL)) {
			prog_phase = -1;
			return -ENODEV;
		}

		ret = ubi_volume_write((char *)target.name, recv_base, 0, recv_off);
		if (ret) {
			printf("ubi_volume_write failed for '%s': %d\n",
			       target.name, ret);
			prog_phase = -1;
			return ret;
		}

		prog_write_done = recv_off;
		prog_done = recv_off;
		prog_phase = 3;
		printf("Flashing complete.\n");
		return 0;
	}

	{
		struct mtd_info *mtd = target.mtd;
		struct erase_info ei = { 0 };
		struct udevice *udev = eth_get_dev();
		struct netif *netif = net_lwip_get_netif();
		loff_t ofs = target.ofs;

		prog_phase = 1;
		prog_done = 0;
		prog_erase_done = 0;
		prog_erase_total = ALIGN(recv_off, mtd->erasesize);
		prog_write_done = 0;
		prog_write_total = recv_off;
		prog_total = prog_erase_total + prog_write_total;

		printf("Erasing and writing %u bytes to '%s'...\n",
		       recv_off, target.name);
		ei.addr = ofs;
		ei.len = ALIGN(recv_off, mtd->erasesize);

		ret = mtd_unlock(mtd, ei.addr, ei.len);
		if (ret && ret != -EOPNOTSUPP) {
			printf("Warning: initial mtd_unlock 0x%llx..+0x%x failed: %d\n",
			       (unsigned long long)ei.addr, (unsigned)ei.len, ret);
		}

		for (loff_t addr = 0; addr < ei.len; addr += mtd->erasesize) {
			struct erase_info e = {
				.addr = ofs + addr,
				.len = mtd->erasesize,
			};
			int tries = 0;

			do {
				ret = mtd_erase(mtd, &e);
				if (!ret)
					break;
				if (ret == -EROFS || ret == -EACCES)
					mtd_unlock(mtd, e.addr, e.len);
				else
					break;
			} while (++tries < 2);

			if (ret) {
				printf("mtd_erase failed: %d\n", ret);
				recovery_release_target(&target);
				prog_phase = -1;
				return ret;
			}

			prog_erase_done = addr + mtd->erasesize;
			if (prog_erase_done > prog_erase_total)
				prog_erase_done = prog_erase_total;
			prog_done = prog_erase_done + prog_write_done;
			if (udev && netif)
				net_lwip_rx(udev, netif);
			WATCHDOG_RESET();
		}

		prog_phase = 2;
		for (u32 written = 0; written < recv_off; ) {
			u32 remain = recv_off - written;
			u32 chunk = remain > (64 * 1024) ? (64 * 1024) : remain;

			ret = mtd_write(mtd, ofs + written, chunk, &retlen,
					recv_base + written);
			if (ret) {
				printf("mtd_write failed: ret=%d at 0x%llx\n",
				       ret, (unsigned long long)(ofs + written));
				recovery_release_target(&target);
				prog_phase = -1;
				return ret;
			}
			if (!retlen) {
				printf("mtd_write made no progress at 0x%llx\n",
				       (unsigned long long)(ofs + written));
				recovery_release_target(&target);
				prog_phase = -1;
				return -EIO;
			}

			written += retlen;
			prog_write_done = written;
			prog_done = prog_erase_done + prog_write_done;
			if (udev && netif)
				net_lwip_rx(udev, netif);
			WATCHDOG_RESET();
		}
	}

	recovery_release_target(&target);
	prog_phase = 3;
	printf("Flashing complete.\n");
	return 0;
}

int run_http_recovery(void)
{
    struct udevice *udev;
    struct netif *netif;
    int rc;

    recv_off = recv_total = 0;
    post_ok = 0;
    upload_done = 0;

    rc = net_lwip_eth_start();
    if (rc < 0) {
        printf("Failed to start Ethernet: %d\n", rc);
        return rc;
    }

    udev = eth_get_dev();
    if (!udev) {
        printf("No active net device\n");
        return -ENODEV;
    }

    netif = net_lwip_new_netif(udev);
    if (!netif)
        return -ENODEV;

    httpd_init();
    printf("HTTP recovery server listening on http://192.168.255.1/\n");

    while (1) {
        if (tstc()) {
            int c = getchar();
            if (c == 0x03) { /* Ctrl-C */
                printf("Abort by user\n");
                break;
            }
        }
        /* net_lwip_rx() already runs sys_check_timeouts(). */
        net_lwip_rx(udev, netif);
        if (flash_request) {
            flash_request = 0;
            printf("Upload done, flashing...\n");
            rc = flash_image();
            if (!rc) {
                printf("Flashing complete. Rebooting in %dms...\n", REBOOT_DELAY_MS);
                reboot_request = 0;
                sys_timeout(REBOOT_DELAY_MS, reboot_delay_cb, NULL);
            }
            else {
                printf("Flashing failed: %d. Keeping server running.\n", rc);
            }
        }
        if (reboot_request) {
            do_reset(NULL, 0, 0, NULL);
        }
        WATCHDOG_RESET();
    }

    net_lwip_remove_netif(netif);
    eth_halt();
    return 0;
}
