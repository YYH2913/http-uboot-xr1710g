# SBE1V1K Chainloader FIT

This builds the stock-`bootm` compatible second-stage chainloader for the
Askey SBE1V1K.

The intended installation path starts from an SBE1V1K eMMC GPT layout. It does
not use the 192.168.33.3/QWRT partition table. Recovery preserves all existing
partitions that end before LBA `0x1b022`, including the boot chain, `0:ART`,
`0:WIFIFW`, and `0:HLOS`; the chainloader is never written to an HLOS slot.
Some QSDK variants omit `0:HLOS_1`, so partition numbers after `0:HLOS` are not
assumed to be stable.

## Serial Boot

Use only temporary U-Boot variables at the stock prompt. Do not run `saveenv`
from stock U-Boot.

```sh
setenv ipaddr 192.168.1.1
setenv serverip 192.168.1.2
tftpboot 0x80000000 sbe1v1k-chainloader.itb
bootm 0x80000000
```

The TFTP path loads the raw FIT at `0x80000000`. The persistent eMMC path loads
the installed FIT at `0x44000000`, and the shim searches both addresses.

The FIT embeds the real second-stage U-Boot as `uboot-1`, but `conf-1` does
not list it as a FIT `loadables` entry. On the SBE1V1K stock U-Boot, preloading
that payload to the second-stage address can overwrite the still-running stock
loader before it jumps to the shim. The shim copies `uboot-1` from the raw FIT
itself after stock `bootm` has transferred control.

If the system TFTP root is not writable, the helper script in this directory
can serve the generated FIT from an unprivileged port:

```sh
board/qualcomm/sbe1v1k-chainloader-fit/serve-chainloader-tftp.py \
	--host 0.0.0.0 \
	--port 6969
```

At startup the helper prints the served FIT path, size, SHA256, and embedded
U-Boot version banner. It also checks `/srv/tftp/sbe1v1k-chainloader.itb` when
serving from a non-default port and warns if stock U-Boot's default TFTP port 69
would see a missing or stale copy. Check the printed hash and banner before
booting from stock U-Boot. The helper also prints the exact temporary stock
U-Boot commands for the selected port.

For the default helper port, those commands are:

```sh
setenv tftpdstp 6969
tftpboot 0x80000000 sbe1v1k-chainloader.itb
setenv tftpdstp
bootm 0x80000000
```

Do not save the environment while using the temporary high-port TFTP path.

## Board Log Analysis

Use the log analyzer on captured serial output before changing EDMA defaults
based on a failed ping:

```sh
board/qualcomm/sbe1v1k-chainloader-fit/analyze-board-log.py board.log
```

By default it compares the log against `include/config/uboot.release`, reports
whether the expected U-Boot banner was seen, checks if short EDMA TX frames were
padded to `hw_len=49`, decodes EDMA misc bits such as `DATA_LEN_ERR` and
`TX_TIMEOUT`, and summarizes TX completion errors and PPE/GMAC TX counters when
they are present in the log. It also reads the latest `PHY<N> Up/Down` lines and
checks that snapshots were captured for the linked PPE port.

The physical LAN1 socket is PPE port 5, backed by QCA8081 address 28 and
UNIPHY1 channel 0. Capture both sides of a failed transaction with:

```text
nss_debug trace on
ping 192.168.255.2
nss_debug snapshot 5
```

Each snapshot now includes compact EDMA, PPE port/queue, MAC, and UNIPHY
samples. With trace enabled, network start records a baseline after EDMA reset;
the post-ping snapshot prints `NSS counters delta`. PPE and MAC pairs are
`frames/bytes`. For a shorter capture, use `nss_debug counters 5` after the
failed ping.

At the host, capture the same transaction on the connected interface:

```sh
sudo ip addr replace 192.168.255.2/24 dev <interface>
sudo ip link set <interface> up
sudo tcpdump -eni <interface> 'arp or icmp'
```

Use the PPE port printed by the latest `PHY<N> Up` line when testing another
socket. Port 3 is LAN2, not LAN1.

## HTTP Migration

Open the recovery page at:

```text
http://192.168.255.1/
```

Choose a partition profile, then enter:

```text
SBE1V1K_REPARTITION
```

The layout selector provides two profiles:

| Profile | Kernel | Root | Data | Chainloader |
| --- | --- | --- | --- | --- |
| `mainline` | `0:HLOS`, 7 MiB | `rootfs`, 122 MiB | `rootfs_data`, 512 MiB | `rsvd_2`, first 4 MiB used |
| `large` | `kernel`, 32 MiB | `rootfs`, 1 GiB | `rootfs_data`, remaining space | `chainloader`, 4 MiB |

The recovery action:

- verifies key boot-chain anchors before the chainloader area;
- rewrites only the area starting at LBA `0x1b022`;
- keeps all existing partitions before LBA `0x1b022` untouched;
- creates the selected tail layout;
- preserves the current FIT from the persistent `0x44000000` or TFTP
  `0x80000000` address, falling back to the active eMMC chainloader partition;
- fully erases the profile-specific chainloader target, then writes the
  preserved FIT into it;
- updates `0:APPSBLENV` with `fw_setenv`-equivalent variable changes and
  verifies them by reading the partition back;
- keeps the HTTP server running so firmware can be uploaded next.

SBE1V1K is an eMMC/GPT device. Recovery writes raw GPT partitions only; it must
not format, create, resize, or write UBI volumes on this board.

Every recovery image update is intentionally destructive. The web client first
posts the decimal image size to `/action/prepare/firmware` or
`/action/prepare/uboot`. The recovery main loop then fully erases `kernel`,
the selected kernel, rootfs, and rootfs_data partitions for firmware, or the
selected chainloader target for a chainloader update. Erase work runs outside
the lwIP POST callback so status requests and TCP maintenance continue while
the operation is in progress.

After `/status` reports `prepared`, the client posts the exact-size image to
`/upload/firmware` or `/upload/uboot`. A direct upload without a matching
prepared target and length is rejected. The HTTP body is written directly to
eMMC through a fixed 1 MiB RAM buffer. Recovery does not retain or validate the
complete upload before writing, so an interrupted or incorrect upload can leave
the target unbootable.
Recovery uses exact eMMC erase/TRIM when the device and partition alignment
allow it. Otherwise, it zero-fills every logical block in the partition so an
erase-group rounding operation cannot overwrite an adjacent GPT partition.
Raw recovery images use the selected profile's `recovery_kernel_pad`: 7 MiB for
`mainline` and 32 MiB for `large`. OpenWrt sysupgrade tar images are parsed as
they arrive; only the `kernel` and `root` members are streamed to their target
partitions, so the complete tar is never staged in RAM. The default upload cap
is 1 GiB, additionally limited by the selected kernel and rootfs capacities.

Selecting `mainline` from an existing `large` installation recreates the
factory-compatible partition entries, but cannot restore old data that the
large rootfs/rootfs_data partitions previously overwrote.

## HTTP Partition Backup

The recovery page lists the current eMMC GPT and can download any partition as
a raw image. `GET /partitions` returns the live partition metadata, and
`GET /backup/partition-N.bin` streams partition number `N` with an exact 64-bit
`Content-Length` and a partition-specific filename.

Backup reads use an aligned buffer of up to 16 MiB; smaller partitions allocate
only their actual size. Therefore, `rootfs_data` and other partitions larger
than 2 GiB do not require a RAM-sized staging image. Backup streams and
destructive upload or repartition requests are mutually exclusive.
Raw factory partitions can contain credentials, calibration data, and
device-specific keys; store the downloaded files securely.

The installed stock U-Boot environment is profile-specific. The relevant
values are:

| Profile | `bootargs` root | Chainloader read |
| --- | --- | --- |
| `mainline` | `/dev/mmcblk0p27` | LBA `0x4f6022`, `0x2000` sectors |
| `large` | `PARTLABEL=rootfs` | LBA `0x1b022`, `0x2000` sectors |

Both profiles set `do_boot=run boot_chainloader`, `do_nothing=true`, and a
three-second interruptible `bootcmd`. The second-stage U-Boot also detects the
current GPT before every boot: it loads `0:HLOS` for `mainline`, or `kernel` for
`large`. HTTP recovery repeats the same detection before selecting firmware and
self-update targets.

`bootcmd` sets `bootargs` again on every boot because the vendor U-Boot resets
it during startup. Other factory U-Boot environment variables are preserved.
The Ethernet MAC fallback also reads `ethaddr` from `0:APPSBLENV` when the
runtime environment does not provide one.

## Large Target Layout

Sector size is 512 bytes.

| Start | Sectors | Size | Name | Purpose |
| ---: | ---: | ---: | --- | --- |
| before 110626 (`0x1b022`) | unchanged | unchanged | existing labels | boot chain, ART, ETHPHYFW, WIFIFW, HLOS, optional HLOS_1 |
| 110626 (`0x1b022`) | 8192 (`0x2000`) | 4 MiB | `chainloader` | raw `sbe1v1k-chainloader.itb` |
| 118818 (`0x1d022`) | 65536 (`0x10000`) | 32 MiB | `kernel` | OpenWrt/QSDK kernel FIT |
| 184354 (`0x2d022`) | 2097152 (`0x200000`) | 1 GiB | `rootfs` | OpenWrt/QSDK root image |
| 2281506 (`0x22d022`) | to last usable sector | about 6.2 GiB | `rootfs_data` | persistent data |

The new layout leaves room for a valid secondary GPT.
The numeric partition index of `chainloader` depends on the source GPT. On
standard factory images it follows `0:HLOS_1`; on QSDK variants without
`0:HLOS_1`, it follows `0:HLOS`. Scripts and recovery code use labels and fixed
LBAs, not the numeric index.

## Mainline Target Layout

The `mainline` profile recreates the original tail boundaries, keeps a valid
secondary GPT, and uses the otherwise empty `rsvd_2` partition for the
chainloader. Important targets are:

| Start | Sectors | Size | Name | Purpose |
| ---: | ---: | ---: | --- | --- |
| 81954 (`0x14022`) | 14336 (`0x3800`) | 7 MiB | `0:HLOS` | OpenWrt kernel FIT |
| 110626 (`0x1b022`) | 249856 | 122 MiB | `rootfs` | OpenWrt root image |
| 610338 (`0x95022`) | 1048576 | 512 MiB | `rootfs_data` | persistent data |
| 5201954 (`0x4f6022`) | 65536 | 32 MiB | `rsvd_2` | raw chainloader FIT; first 4 MiB loaded |

`0:HLOS_1`, `rootfs_1`, and `rootfs_data_1` remain present for partition-number
compatibility. If the source GPT omitted `0:HLOS_1`, migration creates the
missing entry so mainline's `/dev/mmcblk0p27` root argument stays valid.

## Recovery Targets

Recovery selects these targets from the detected GPT:

| Profile | Chainloader | Firmware | Root data |
| --- | --- | --- | --- |
| `mainline` | `0#rsvd_2` | `0#0:HLOS` + `0#rootfs` | `0#rootfs_data` |
| `large` | `0#chainloader` | `0#kernel` + `0#rootfs` | `0#rootfs_data` |

If loading or booting the selected kernel partition fails and `bootm` returns
to U-Boot, the second-stage chainloader starts `http_recovery` automatically.

## Offline eMMC Recovery

If the installed chainloader hangs before the second-stage U-Boot banner and
the stock U-Boot shell cannot be reached, rewrite only the profile-specific
chainloader target in the eMMC user area. Do not rewrite the full device and do
not use the eMMC boot0/boot1 hardware partitions.

| Profile | Partition label | Start LBA | Byte offset | Written size |
| --- | --- | ---: | ---: | ---: |
| `mainline` | `rsvd_2` | 5201954 (`0x4f6022`) | `0x9ec04400` | 4 MiB |
| `large` | `chainloader` | 110626 (`0x1b022`) | `0x3604400` | 4 MiB |

After attaching the eMMC to a Linux host, first confirm the device and GPT:

```sh
sudo sgdisk -p /dev/sdX
```

Confirm the selected label and start LBA before writing. If the host creates a
partition node, write the padded 4 MiB chainloader image to that node:

```sh
sudo dd if=sbe1v1k-chainloader/sbe1v1k-chainloader-partition.img \
	of=/dev/sdXN bs=4M conv=fsync
```

If a partition node is not available, write the same image at the fixed byte
offset on the whole eMMC user device:

```sh
sudo dd if=sbe1v1k-chainloader/sbe1v1k-chainloader-partition.img \
	of=/dev/sdX bs=512 seek=<profile-start-LBA> conv=notrunc,fsync
```

For `/dev/mmcblkN`, partition nodes use the `/dev/mmcblkNpM` form. Replace all
device and partition placeholders with the values confirmed from the live GPT.

If a full-device rewrite is unavoidable, first make a raw backup of the eMMC
user area and, when exposed by the reader, both eMMC boot areas:

```sh
sudo dd if=/dev/sdX of=sbe1v1k-emmc-user-before.img \
	bs=4M conv=sync,noerror status=progress
sudo dd if=/dev/mmcblkNboot0 of=sbe1v1k-emmc-boot0-before.img \
	bs=4M conv=sync,noerror status=progress
sudo dd if=/dev/mmcblkNboot1 of=sbe1v1k-emmc-boot1-before.img \
	bs=4M conv=sync,noerror status=progress
```

The factory layout has known integration problems that this chainloader flow
intentionally replaces: the tail partition can invalidate the backup GPT, the
stock environment does not boot the new GPT layout, and the stock loader cannot
directly load the embedded second-stage U-Boot payload safely. Those are not
reasons to overwrite board-unique data.

For whole-device recovery, preserve these partitions from the same board unless
there is a verified same-unit backup:

```text
0:APPSBLENV  environment, fallback MAC variables
0:ART        board calibration and MAC-related data
0:LICENSE    device license data
```

The boot-chain partitions before `chainloader` can be restored from a trusted
same-model factory image only when they are known bad. Do not use another
unit's `0:ART`, `0:APPSBLENV`, or `0:LICENSE` as a generic replacement.

## Build

```sh
make sbe1v1k_chainloader_defconfig
make CROSS_COMPILE=aarch64-linux-gnu- -j8
board/qualcomm/sbe1v1k-chainloader-fit/build-chainloader-fit.sh \
	--payload u-boot.bin \
	--outdir sbe1v1k-chainloader
```

Default output:

- `sbe1v1k-chainloader/sbe1v1k-chainloader.itb`
- `sbe1v1k-chainloader/sbe1v1k-chainloader-partition.img`
- `sbe1v1k-chainloader/sbe1v1k-chainloader-hlos.elf`
- `sbe1v1k-chainloader/sbe1v1k-chainloader-shim.bin`
- `sbe1v1k-chainloader/sbe1v1k-chainloader-control.dtb`

The partition image contains the raw FIT followed by zeroes to fill the complete
4 MiB `chainloader` partition. Use it only for offline full-partition writes;
HTTP recovery and TFTP use the raw `.itb`. The build script prints SHA256 values
for these artifacts and the embedded U-Boot banner found in the generated raw
FIT. Check that banner before copying or serving the FIT.

The `*-hlos.elf` wrapper is generated only for inspection and experiments with
the original Askey loader format. The normal chainloader path is the raw FIT.
