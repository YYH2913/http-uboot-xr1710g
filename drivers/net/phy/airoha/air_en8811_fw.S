/* SPDX-License-Identifier: GPL-2.0+ */

.section .rodata.en8811h_fw,"a"
.balign 8
.global en8811h_fw_lzma_begin
.type en8811h_fw_lzma_begin, %object
en8811h_fw_lzma_begin:
.incbin EN8811H_FW_FILE
.global en8811h_fw_lzma_end
.type en8811h_fw_lzma_end, %object
en8811h_fw_lzma_end:
.balign 8
