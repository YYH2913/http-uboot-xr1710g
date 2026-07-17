# Askey SBE1V1K Chainloader and HTTP Recovery

This project provides a stock-`bootm` compatible second-stage U-Boot and an eMMC/GPT recovery environment for the Askey SBE1V1K. The board uses a Qualcomm IPQ9570 SoC, 1 GiB of RAM, and an eMMC boot device.

The chainloader solves three board-specific problems:

- the stock U-Boot cannot safely load the current U-Boot payload directly;
- OpenWrt mainline and the large-storage installation use different GPT layouts and boot arguments;
- firmware images can be too large to stage completely in RAM.

> **Warning:** Firmware and chainloader updates are intentionally destructive. The HTTP server erases the selected partitions before it receives the image body. Power loss, a network interruption, or an incorrect image can leave the active system unbootable. Make a per-device backup and retain serial access to the stock U-Boot before changing the GPT or flashing an image.

## Features

| Feature                     | Implementation                                               |
| --------------------------- | ------------------------------------------------------------ |
| Stock `bootm` compatibility | A small AArch64 shim is loaded as the FIT kernel; the shim locates, copies, and starts the real `u-boot.bin` payload |
| Dual-layout boot            | U-Boot and HTTP recovery detect and support the `mainline` and `large` GPT profiles |
| HTTP recovery               | Static address `192.168.255.1` with an integrated DHCP helper that offers `192.168.255.2` to a directly connected host |
| Streamed firmware writes    | Targets are erased first, then the request body is written through a fixed 1 MiB buffer without retaining the complete image in RAM |
| OpenWrt image support       | Accepts sysupgrade tar images with `kernel` and `root` members, plus profile-matched raw recovery images |
| Chainloader self-update     | Automatically selects `rsvd_2` or `chainloader` from the detected layout; the accepted FIT is limited to 4 MiB |
| GPT migration               | The web UI can rebuild the tail as `mainline` or `large` while preserving validated prefix partitions |
| Partition backup            | Downloads `boot0`, `boot1`, or any GPT partition individually, or streams all readable partitions into one tar archive |
| Multi-rate Ethernet         | The current NSS/PPE path supports QCA8075 1G, QCA8081 2.5G, and RTL8261BE multi-rate/10G PHYs |
| Recovery status LEDs        | Hardware PWM reports preparation, erase, write, completion, and error states |
| Serial diagnostics          | Board-log analysis and NSS/PPE/EDMA counter snapshots are available for network debugging |

## First Installation

### 1. Temporarily Boot the Chainloader FIT

Configure the host as `192.168.1.2/24` and place `sbe1v1k-chainloader.itb` in the TFTP root. At the stock U-Boot prompt, use only temporary variables:

```sh
setenv ipaddr 192.168.1.1
setenv serverip 192.168.1.2
tftpboot 0x80000000 sbe1v1k-chainloader.itb
bootm 0x80000000
```

Do not run `saveenv`. TFTP places the raw FIT at `0x80000000`. The persistent stock boot path later reads the installed FIT from eMMC to `0x44000000`; the shim recognizes both addresses.

### 2. Start HTTP Recovery and Back Up the Device

If no valid firmware is present, second-stage U-Boot enters HTTP recovery after the boot attempt fails. To start it explicitly:

```text
http_recovery
```

Keep the connected host on DHCP, or configure it as `192.168.255.2/24` with no gateway. When U-Boot reports that the recovery server is listening, open:

```text
http://192.168.255.1/
```

![Backup page during first installation](board/qualcomm/sbe1v1k-chainloader-fit/images/recovery-backup.png)

Before changing the layout, backing up GPT partitions `p1` through `p26` and both eMMC hardware boot partitions is mandatory. The eMMC specification calls the hardware areas Boot Partition 1 and Boot Partition 2; U-Boot and Linux expose them as `boot0` and `boot1`, and the archive stores them as `emmc-boot0.img` and `emmc-boot1.img`.

The recommended method is **Download all (.tar)** on the Backup page. The one-click archive includes `boot0`, `boot1`, and every valid GPT partition, so `p1` through `p26` are included automatically. A complete download typically takes about 1.5 hours; keep the recovery server, network connection, browser, and destination storage running until it finishes.

The web archive is partition-level and does not contain the user-area GPT headers or unallocated sectors. For a complete sector-level image that includes those regions, use an external eMMC reader.

### 3. Select and Apply a Layout

Open the eMMC layout page:

1. Select `mainline` or `large`.

2. Enter the exact confirmation token:

   ```text
   SBE1V1K_REPARTITION
   ```

3. Apply the layout and wait for GPT, chainloader, and APPSBLENV verification.

4. Do not power off or reboot. Upload matching OpenWrt firmware next.

Migration preserves the currently running FIT, so the first installation does not require a separate upload of `sbe1v1k-chainloader-partition.img`.

### 4. Upload OpenWrt Firmware

The preferred input is the matching device's OpenWrt sysupgrade tar. Its file name usually still ends in `.bin`; the page identifies the tar from its content.

| Image format           | Write behavior                                               |
| ---------------------- | ------------------------------------------------------------ |
| OpenWrt sysupgrade tar | Parse the tar while receiving it and write only `kernel` and `root` into the current profile's targets |
| Raw recovery image     | `mainline`: first 7 MiB to `0:HLOS`, remainder to `rootfs`; `large`: first 32 MiB to `kernel`, remainder to `rootfs` |

The browser first posts the image length. U-Boot erases kernel, rootfs, and rootfs_data in its main loop, reports `prepared`, then accepts a second request with the exact body length. Direct uploads without matching preparation are rejected.

The default firmware cap is 1 GiB and is further constrained by the live kernel/rootfs capacities. A raw image must place the rootfs at the correct 7 MiB or 32 MiB boundary; raw images cannot be reused across profiles.

## Recovery Web Interface

Run `http_recovery` at the second-stage U-Boot prompt, then open:

```text
http://192.168.255.1/
```

The integrated DHCP helper normally gives the connected computer `192.168.255.2/24`. If DHCP is unavailable, configure that address manually and leave the gateway empty.

The screenshots below are rendered from the current embedded `index.html` with representative `large`-layout metadata. No destructive operation is active in the captures.

### Shared Controls and Status

The sidebar selects one of four operation pages. The header of every page shows the current target, operation mode, and size or boundary limit. The state badge reports `Idle`, `Ready`, `Preparing`, `Erasing`, `Writing`, `Done`, or an error state as appropriate.

Firmware and Chainloader operations use three progress rows:

- **Upload** reports browser-to-U-Boot transfer progress.
- **Erase** reports target preparation performed by the U-Boot main loop.
- **Write** reports bytes committed to eMMC.

Navigation and conflicting controls are disabled while a destructive request is active. Firmware writes, chainloader writes, layout migration, and backup streams are mutually exclusive on the server.

### Firmware Page

![Firmware recovery page](board/qualcomm/sbe1v1k-chainloader-fit/images/recovery-firmware.png)

The **Firmware** page installs an OpenWrt system image into the active profile.

- **Target** changes automatically between `0:HLOS + rootfs` for `mainline` and `kernel + rootfs` for `large`.
- **Choose file** accepts `.bin`, `.img`, `.itb`, or `.tar`; the browser checks the file header instead of relying only on the extension.
- An OpenWrt sysupgrade tar is accepted when it contains usable `kernel` and `root` members.
- A raw recovery image must contain a FIT at the beginning and a SquashFS root at the profile boundary: 7 MiB for `mainline`, or 32 MiB for `large`.
- The displayed size limit is derived from the live partition capacities and the default 1 GiB upload cap.
- Pressing **Upload firmware** first submits the image length. U-Boot erases kernel, rootfs, and rootfs_data outside the POST callback. Only after the status becomes `prepared` does the browser send the image body.
- A successful write schedules a reboot. An interrupted write has no rollback.

For a streamed sysupgrade tar, U-Boot parses each 512-byte tar header as it arrives and writes only the required members. For a raw image, it writes the fixed kernel span first and continues with the rootfs target.

### Chainloader Page

![Chainloader recovery page](board/qualcomm/sbe1v1k-chainloader-fit/images/recovery-chainloader.png)

The **Chainloader** page updates this second-stage U-Boot installation.

- It accepts the raw `sbe1v1k-chainloader.itb`, not `u-boot.bin`, an OpenWrt image, or the inspection-only HLOS wrapper.
- The maximum accepted image is 4 MiB.
- `mainline` selects `0#rsvd_2`; `large` selects `0#chainloader`.
- The complete target partition is erased before the FIT body is streamed.
- Other GPT partitions are not modified by this page.
- Losing power after preparation can remove the installed recovery boot path.

The padded `sbe1v1k-chainloader-partition.img` is intended for offline writes; the HTTP page should use the smaller raw `.itb`.

### eMMC Layout Page

![eMMC layout migration page](board/qualcomm/sbe1v1k-chainloader-fit/images/recovery-layout.png)

The **eMMC layout** page changes the supported partition profile.

- **Partition profile** selects either **OpenWrt mainline / factory-compatible** or **Large storage**.
- The live table shows the target labels, start LBAs, sizes, and purpose.
- Both profiles use LBA `110626` as the preserved-prefix boundary.
- The warning describes which tail definition will be replaced and which firmware must be uploaded before rebooting.
- The action remains disabled until the exact token `SBE1V1K_REPARTITION` is entered.
- The migration verifies fixed factory anchors, preserves the running FIT, writes and verifies the new GPT, reinstalls the FIT in the new target, and updates and verifies `0:APPSBLENV`.
- The HTTP server remains active after migration so matching firmware can be installed before rebooting.

Migration rewrites partition definitions and boot configuration. It cannot restore data that an earlier layout already overwrote.

### Backup Page

![Partition backup page](board/qualcomm/sbe1v1k-chainloader-fit/images/recovery-backup.png)

The **Backup** page is read-only.

- The selector is populated from `GET /partitions` and includes both eMMC hardware boot areas, Boot Partition 1 and Boot Partition 2, exposed as `boot0` and `boot1`, plus every valid GPT partition in the user area.
- **Download partition** streams the selected source as a raw `.img` file with an exact 64-bit `Content-Length`.
- **Download all (.tar)** requests `GET /backup/all.tar`. U-Boot generates a standard uncompressed ustar containing both hardware boot areas as `emmc-boot0.img` and `emmc-boot1.img`, followed by one `pNN-name.img` for every GPT partition.
- Before the first layout migration, retaining `p1` through `p26`, `emmc-boot0.img`, and `emmc-boot1.img` is mandatory. The one-click archive contains all of them.
- The complete archive is never staged in RAM or written to a temporary eMMC partition. Backup reads use at most a 16 MiB DMA-aligned buffer.
- RPMB is excluded because it is an authenticated eMMC region, not an ordinary linear block partition.
- The tar does not include the user-area GPT headers or unallocated sectors. Use an external reader and image the complete user area when those are needed.
- Factory-sized full backups exceed 7 GiB and typically take about 1.5 hours. The destination filesystem must have enough free space and must not be FAT32.

Raw backups can contain MAC addresses, calibration data, keys, and license material. Store them as device-specific secrets and calculate an external SHA256 for important archives.

## Safety Boundaries

- This project does not stage the complete firmware before writing and does not implement atomic A/B updates.
- HTTP recovery checks the selected target, image structure, exact request length, and partition boundaries. It does not authenticate a complete image signature or verify an end-to-end content hash.
- `mainline` retains `0:HLOS_1`, `rootfs_1`, and `rootfs_data_1` for factory numbering compatibility. The current updater operates only on the active `0:HLOS`, `rootfs`, and `rootfs_data`; it does not switch to the alternate slot.
- `large` is also a single-active-system layout, not A/B.
- Recovery operates on raw GPT partitions in the eMMC user area. It does not create, resize, or write UBI volumes on this board.
- `boot0` and `boot1` are exposed only through read-only backup endpoints. Normal installation, updates, and offline recovery must not write the chainloader into either eMMC hardware boot area.
- RPMB is deliberately excluded from backup and write operations.
- Do not run `saveenv` in the stock U-Boot. The migration code updates only the required `0:APPSBLENV` variables while preserving all other entries.

## Build and Artifacts

Run from the U-Boot repository root:

```sh
make sbe1v1k_chainloader_defconfig
make CROSS_COMPILE=aarch64-linux-gnu- -j8
board/qualcomm/sbe1v1k-chainloader-fit/build-chainloader-fit.sh \
	--payload u-boot.bin \
	--outdir sbe1v1k-chainloader
```

The packaging script prints a SHA256 for every artifact and the U-Boot version embedded in the FIT. Confirm that the release banner does not contain `dirty` before serving or flashing it.

| File                                | Purpose                                            | Correct use                                                  |
| ----------------------------------- | -------------------------------------------------- | ------------------------------------------------------------ |
| `sbe1v1k-chainloader.itb`           | Stock U-Boot TFTP boot and HTTP Chainloader update | Use as the raw FIT for TFTP or HTTP; do not use it as an offline full-partition image |
| `sbe1v1k-chainloader-partition.img` | Raw FIT padded with zeroes to exactly 4 MiB        | Use only for an offline write to the correct chainloader storage area |
| `sbe1v1k-chainloader-hlos.elf`      | Askey HLOS wrapper for inspection and experiments  | Not used by the normal installation path                     |
| `sbe1v1k-chainloader-shim.bin`      | First-stage shim embedded in the FIT               | Build intermediate; do not flash directly                    |
| `sbe1v1k-chainloader-control.dtb`   | Control DTB embedded in the FIT                    | Build intermediate; do not flash directly                    |
| `u-boot.bin`                        | The actual second-stage U-Boot payload             | Never pass directly to stock `bootm` or write directly to eMMC |

## Flashing Workflow Summary

| Scenario                    | File or action                                               | Actual destination                                           |
| --------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| First installation          | TFTP-boot `sbe1v1k-chainloader.itb` from stock U-Boot, then run a layout migration in the web UI | Migration installs the running FIT and updates `0:APPSBLENV`; do not write an HLOS slot manually |
| HTTP second-stage update    | Upload raw `sbe1v1k-chainloader.itb` on the Chainloader page | `mainline` writes `rsvd_2`; `large` writes `chainloader`     |
| HTTP OpenWrt update         | Upload the matching sysupgrade tar or a raw recovery image for the current profile | Writes the active kernel/rootfs targets and erases rootfs_data |
| Offline second-stage repair | Write `sbe1v1k-chainloader-partition.img` with `dd`          | Write the first 4 MiB of `rsvd_2` for `mainline`, or the complete `chainloader` partition for `large` |
| Full eMMC restoration       | Use verified user-area, boot0, and boot1 images from the same device | Last resort only; normal installation does not require a full-device write |

Never write `u-boot.bin` directly to eMMC. Never install the chainloader in `0:HLOS`, `0:HLOS_1`, boot0, or boot1.

## Supported Partition Layouts

All LBAs below use 512-byte sectors. HTTP recovery selects partitions by label and validates fixed LBAs instead of depending on unstable numeric partition indices.

### Layout Comparison

| Profile    | Kernel           | Rootfs            | Data                                              | Chainloader                                | Linux root argument |
| ---------- | ---------------- | ----------------- | ------------------------------------------------- | ------------------------------------------ | ------------------- |
| `mainline` | `0:HLOS`, 7 MiB  | `rootfs`, 122 MiB | `rootfs_data`, 512 MiB                            | `rsvd_2`, with only the first 4 MiB loaded | `/dev/mmcblk0p27`   |
| `large`    | `kernel`, 32 MiB | `rootfs`, 1 GiB   | `rootfs_data`, all remaining space, about 6.2 GiB | `chainloader`, 4 MiB                       | `PARTLABEL=rootfs`  |

### Compatibility Matrix

| Existing media layout                                        | Detection          | Required action                                              |
| ------------------------------------------------------------ | ------------------ | ------------------------------------------------------------ |
| Standard factory GPT, or an installed OpenWrt mainline layout | `mainline`         | A first installation must still run the desired profile migration. Select `mainline` to retain mainline boundaries, or `large` to convert to large storage |
| Installed large-storage layout                               | `large`            | Boots directly; firmware and chainloader updates select large-layout targets automatically |
| Some QSDK factory variants without `0:HLOS_1`                | Normally `unknown` | Migration is allowed when the fixed boot-chain anchors match. A `mainline` migration adds the missing `0:HLOS_1` entry |
| Labels, starts, or capacities do not match either descriptor | `unknown`          | Do not upload firmware directly. Back up the device and try migration; an anchor-verification failure means the layout is unsupported |
| NAND/UBI or `192.168.33.3`/QWRT layout                       | Unsupported        | Do not use this project's GPT operations or offline offsets  |

### `large` Profile

Migration preserves all existing partitions ending at or before LBA `110626` (`0x1b022`). This includes the boot chain, `0:ART`, `0:ETHPHYFW`, `0:WIFIFW`, `0:HLOS`, and `0:HLOS_1` when present. It then rebuilds the tail:

|            Start LBA |              Sectors |          Size | Name            | Purpose                                               |
| -------------------: | -------------------: | ------------: | --------------- | ----------------------------------------------------- |
|           `< 110626` |            unchanged |     unchanged | existing labels | Boot chain, calibration, PHY/Wi-Fi firmware, and HLOS |
|   110626 (`0x1b022`) |      8192 (`0x2000`) |         4 MiB | `chainloader`   | Raw `sbe1v1k-chainloader.itb`                         |
|   118818 (`0x1d022`) |    65536 (`0x10000`) |        32 MiB | `kernel`        | OpenWrt/QSDK kernel FIT                               |
|   184354 (`0x2d022`) | 2097152 (`0x200000`) |         1 GiB | `rootfs`        | SquashFS/root image                                   |
| 2281506 (`0x22d022`) |   to last usable LBA | about 6.2 GiB | `rootfs_data`   | F2FS overlay or persistent data                       |

The layout reserves the final 33 sectors for a valid secondary GPT. The numeric partition index of `chainloader` depends on whether the source GPT contained `0:HLOS_1`, so code and recovery instructions use the label and fixed LBA.

### `mainline` Profile

This profile rebuilds the factory-compatible tail and uses the otherwise empty `rsvd_2` partition as the chainloader container:

|            Start LBA |          Sectors |    Size | Name          | Purpose                                                 |
| -------------------: | ---------------: | ------: | ------------- | ------------------------------------------------------- |
|    81954 (`0x14022`) | 14336 (`0x3800`) |   7 MiB | `0:HLOS`      | OpenWrt kernel FIT                                      |
|   110626 (`0x1b022`) |           249856 | 122 MiB | `rootfs`      | OpenWrt root image                                      |
|   610338 (`0x95022`) |          1048576 | 512 MiB | `rootfs_data` | Overlay or persistent data                              |
| 5201954 (`0x4f6022`) |            65536 |  32 MiB | `rsvd_2`      | Raw chainloader FIT; stock U-Boot reads the first 4 MiB |

`0:HLOS_1`, `rootfs_1`, and `rootfs_data_1` are retained or recreated so that the mainline root remains `/dev/mmcblk0p27`. They are not active alternate slots for the current recovery updater.

### Layout Detection and Migration

HTTP recovery first verifies `large`, then `mainline`:

1. It checks fixed partition labels, start LBAs, and capacities, including the known `0:HLOS` anchor.
2. `large` must match `chainloader`, `kernel`, `rootfs`, and `rootfs_data`.
3. `mainline` must match `0:HLOS_1`, p27 `rootfs`, `rootfs_data`, and `rsvd_2`.
4. A match sets the profile-specific kernel, rootfs, data, chainloader, root argument, and `recovery_kernel_pad` values.
5. If neither descriptor matches, recovery reports `unknown`. Do not rely on default targets to flash firmware in that state.

The layout migration uses a separate set of prefix safety checks:

1. Verify fixed boot-chain anchors including `0:SBL1`, `0:APPSBLENV`, `0:APPSBL`, `0:ART`, `0:ETHPHYFW`, `0:WIFIFW`, and `0:HLOS`.
2. Preserve partition definitions before LBA `110626` and preserve the disk GUID.
3. Build the selected tail, run `gpt write`, and read the result back for exact verification.
4. Preserve the active chainloader FIT from persistent address `0x44000000`, TFTP address `0x80000000`, or the current eMMC target.
5. Erase the new chainloader target and reinstall the preserved FIT.
6. Update `bootargs`, `boot_chainloader`, `do_boot`, `do_nothing`, and `bootcmd` in `0:APPSBLENV`, recalculate its CRC, and verify a complete readback.

Converting from `large` back to `mainline` restores partition definitions only. It cannot restore factory tail data that was previously overwritten.

## Updating an Installed System

### Update OpenWrt

Enter HTTP recovery, open Firmware, and upload a sysupgrade tar or a raw recovery image for the detected profile. The active kernel, rootfs, and rootfs_data are erased before the body is accepted.

Whether ordinary OpenWrt `sysupgrade` works from the running OS depends on the platform upgrade scripts included in that firmware. This document describes the second-stage U-Boot HTTP path.

### Update Second-Stage U-Boot

Enter HTTP recovery, open Chainloader, and upload raw `sbe1v1k-chainloader.itb`:

| Profile    | Automatically selected target                                |
| ---------- | ------------------------------------------------------------ |
| `mainline` | `0#rsvd_2`; erase the complete partition and write the FIT at its start |
| `large`    | `0#chainloader`; erase the complete partition and write the FIT |

Do not upload `u-boot.bin`, `*-hlos.elf`, or OpenWrt firmware on this page. The 4 MiB `*-partition.img` is reserved for offline full-area writes.

## Offline eMMC Recovery

If the installed chainloader fails before the second-stage U-Boot banner and the stock shell is no longer reachable, rewrite only the profile-specific chainloader target in the eMMC user area. Do not rewrite the complete device and do not write boot0 or boot1.

| Profile    | Label         |            Start LBA |  Byte offset | Offline write length |
| ---------- | ------------- | -------------------: | -----------: | -------------------: |
| `mainline` | `rsvd_2`      | 5201954 (`0x4f6022`) | `0x9ec04400` |                4 MiB |
| `large`    | `chainloader` |   110626 (`0x1b022`) |  `0x3604400` |                4 MiB |

After attaching eMMC to a Linux host, verify the device and live GPT:

```sh
sudo sgdisk -p /dev/sdX
lsblk -o NAME,SIZE,START,PARTLABEL /dev/sdX
```

If the correct partition node exists, write the padded 4 MiB image to it:

```sh
sudo dd if=sbe1v1k-chainloader/sbe1v1k-chainloader-partition.img \
	of=/dev/sdXN bs=4M conv=fsync status=progress
```

If no partition node is available, use the verified start LBA on the complete eMMC user device:

```sh
sudo dd if=sbe1v1k-chainloader/sbe1v1k-chainloader-partition.img \
	of=/dev/sdX bs=512 seek=<profile-start-LBA> conv=notrunc,fsync \
	status=progress
```

For `/dev/mmcblkN`, partition nodes use `/dev/mmcblkNpM`. Never copy a numeric partition index from this document; cross-check both `PARTLABEL` and start LBA against the attached device.

Before any unavoidable full-device restoration, save the user area and both hardware boot areas when the reader exposes them:

```sh
sudo dd if=/dev/sdX of=sbe1v1k-emmc-user-before.img \
	bs=4M conv=sync,noerror status=progress
sudo dd if=/dev/mmcblkNboot0 of=sbe1v1k-emmc-boot0-before.img \
	bs=4M conv=sync,noerror status=progress
sudo dd if=/dev/mmcblkNboot1 of=sbe1v1k-emmc-boot1-before.img \
	bs=4M conv=sync,noerror status=progress
```

The factory tail reaches sectors needed by the secondary GPT, which is one reason migration rebuilds the tail. It is not a reason to overwrite unique per-device data in `0:ART`, `0:APPSBLENV`, or `0:LICENSE`.

## Boot Chain Implementation

Stock U-Boot `bootm` handles a FIT kernel and FDT, but listing the real second-stage U-Boot as a FIT `loadables` entry can preload it over memory still used by the running stock loader. `conf-1` therefore declares only the shim kernel and control DTB:

```text
stock XBL
  -> stock U-Boot bootm
     -> chainloader shim
        -> locate uboot-1 in the raw FIT
           -> copy it to 0x4a240000
              -> enter second-stage U-Boot
```

The TFTP FIT is at `0x80000000`; the persistent FIT is loaded at `0x44000000`. Only after stock `bootm` transfers control does the shim copy `uboot-1` from the raw FIT, avoiding an early overwrite of the stock loader.

Migration writes profile-specific stock environment values:

| Profile    | `bootargs` root    | `boot_chainloader` read          |
| ---------- | ------------------ | -------------------------------- |
| `mainline` | `/dev/mmcblk0p27`  | LBA `0x4f6022`, `0x2000` sectors |
| `large`    | `PARTLABEL=rootfs` | LBA `0x1b022`, `0x2000` sectors  |

Both profiles set `do_boot=run boot_chainloader`, `do_nothing=true`, and an interruptible three-second `bootcmd`. Stock U-Boot resets `bootargs` during startup, so `bootcmd` assigns it again before every chainloader boot.

Second-stage `detect_layout` checks for the `kernel` label before every normal boot. If present, it loads the `large` kernel; otherwise it loads mainline `0:HLOS`. If `bootm` returns, it starts HTTP recovery automatically. HTTP recovery uses the stricter geometry verification described above.

## Streamed Writes and Boundary Enforcement

Firmware and chainloader HTTP updates follow this sequence:

```text
browser submits format and decimal length
  -> U-Boot resolves the detected profile and target capacities
  -> U-Boot fully erases the target partitions
  -> status changes to prepared
  -> browser sends an exact-length body
  -> U-Boot writes through a 1 MiB DMA buffer
  -> U-Boot checks received and per-target byte counts
```

A raw firmware stream crosses from the fixed kernel span to rootfs at the profile boundary. A sysupgrade stream parses each 512-byte tar header and only accepts the required kernel/root entries. Every write is constrained to the resolved partition start and end LBAs.

Erase prefers exact eMMC erase/TRIM when partition and erase-group alignment make it safe. If erase-group rounding could affect an adjacent partition, the implementation zero-fills every logical block in the target instead.

This design uses more eMMC I/O but avoids holding a large image in 1 GiB RAM. It also means that the active system is already damaged once preparation has finished; a failed body transfer cannot roll back automatically.

## Partition Backup Implementation

Dynamic backup endpoints are:

```text
GET /partitions
GET /backup/boot0.bin
GET /backup/boot1.bin
GET /backup/partition-N.bin
GET /backup/all.tar
```

Individual and complete backups use at most a 16 MiB DMA-aligned buffer. `all.tar` first scans source metadata to calculate an exact 64-bit `Content-Length`, then generates standard ustar output as the socket consumes it:

```text
member header -> raw partition data -> 512-byte padding -> next member
```

The member order is eMMC Boot Partition 1 (`boot0`), eMMC Boot Partition 2 (`boot1`), then every valid GPT partition, followed by two 512-byte zero terminators. The complete tar never exists in RAM or an eMMC staging area. Closing, completing, or failing a stream restores eMMC user hardware partition 0 before allowing a destructive request.

The tar checksum covers each tar header, not raw partition content. A device read error truncates the HTTP response. Check the downloaded size and calculate an external SHA256 for archival copies.

## Ethernet and Recovery Network

| Physical label | PPE port | PHY                 | Maximum rate | Interface mode |
| -------------- | -------: | ------------------- | -----------: | -------------- |
| LAN2           |        3 | QCA8075 address 18  |           1G | QSGMII         |
| LAN3           |        4 | QCA8075 address 19  |           1G | QSGMII         |
| LAN1           |        5 | QCA8081 address 28  |         2.5G | USXGMII        |
| WAN            |        6 | RTL8261BE address 0 |          10G | USXGMII        |

Actual negotiated rate depends on the peer and cable. Recovery allows Ethernet initialization with no active link and uses ports that configure successfully and report link-up. The Ethernet MAC comes from the runtime environment when available, with an `ethaddr` fallback from `0:APPSBLENV`.

## Upstream References


| Public source | Referenced implementation |
| --- | --- |
| [U-Boot upstream base `a7830e87555a`](https://github.com/u-boot/u-boot/commit/a7830e87555abfb81cc69275cecb2bc0fbde5b28) | Base `bootm`, FIT, eMMC/GPT, PHY, and lwIP networking infrastructure |
| [OpenWrt PR #21586: Askey SBE1V1K support](https://github.com/openwrt/openwrt/pull/21586) | Board DTS, Ethernet port and PHY topology, factory/mainline partition geometry, boot environment behavior, and OpenWrt image definitions |
| [OpenWrt PR #24033: IPQ9574 USXGMII fixes](https://github.com/openwrt/openwrt/pull/24033) | Public IPQ9574 PCS/UNIPHY mode programming, reset and clock sequencing, and USXGMII in-band autonegotiation |
| [Linux upstream QCA808x PHY driver](https://github.com/torvalds/linux/blob/master/drivers/net/phy/qcom/qca808x.c) | QCA8081 2.5G advertisement and SerDes FIFO link-state handling |
| [OpenWrt commit `6369c9e5c799`: Realtek 5G/10G PHY support](https://github.com/openwrt/openwrt/commit/6369c9e5c79994c380d0c63cfb003c935a974332) | Public RTL8261BE/RTL8261N identification, initialization patch engine, firmware-table handling, and link status logic |