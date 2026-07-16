#!/usr/bin/env python3
"""Regenerate fsdata_xr1710g.c from htdocs/ in lwIP makefsdata format.

Matches the existing file layout: raw file data (no embedded HTTP
headers, LWIP_HTTPD_DYNAMIC_HEADERS builds them), flags 0, file name
padded with NULs to a multiple of 4.
"""
import os

HTDOCS = os.path.join(os.path.dirname(__file__), "htdocs")
OUT = os.path.join(os.path.dirname(__file__), "fsdata_xr1710g.c")

HEADER = '''#include "lwip/apps/fs.h"
#include "lwip/def.h"


#define file_NULL (struct fsdata_file *) NULL


#ifndef FS_FILE_FLAGS_HEADER_INCLUDED
#define FS_FILE_FLAGS_HEADER_INCLUDED 1
#endif
#ifndef FS_FILE_FLAGS_HEADER_PERSISTENT
#define FS_FILE_FLAGS_HEADER_PERSISTENT 0
#endif
/* FSDATA_FILE_ALIGNMENT: 0=off, 1=by variable, 2=by include */
#ifndef FSDATA_FILE_ALIGNMENT
#define FSDATA_FILE_ALIGNMENT 0
#endif
#ifndef FSDATA_ALIGN_PRE
#define FSDATA_ALIGN_PRE
#endif
#ifndef FSDATA_ALIGN_POST
#define FSDATA_ALIGN_POST
#endif
#if FSDATA_FILE_ALIGNMENT==2
#include "fsdata_alignment.h"
#endif
'''


def hex_lines(data):
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        lines.append(",".join("0x%02x" % b for b in chunk) + ",")
    return "\n".join(lines)


def c_ident(name):
    return name.replace("/", "_").replace(".", "_")


def main():
    files = sorted(f for f in os.listdir(HTDOCS) if not f.startswith("."))
    parts = [HEADER]
    entries = []
    for name in files:
        uri = "/" + name
        ident = c_ident(uri.lstrip("/"))
        raw = open(os.path.join(HTDOCS, name), "rb").read()
        namebuf = uri.encode() + b"\0"
        while len(namebuf) % 4:
            namebuf += b"\0"
        parts.append("#if FSDATA_FILE_ALIGNMENT==1\n"
                     "static const unsigned int dummy_align__%s = %d;\n"
                     "#endif\n" % (ident, len(entries)))
        parts.append("static const unsigned char FSDATA_ALIGN_PRE data__%s[]"
                     " FSDATA_ALIGN_POST = {\n" % ident)
        parts.append("/* %s (%d chars) */\n%s\n\n" %
                     (uri, len(uri), hex_lines(namebuf)))
        parts.append("/* raw file data (%d bytes) */\n%s};\n\n" %
                     (len(raw), hex_lines(raw)))
        entries.append((ident, len(namebuf)))

    prev = "file_NULL"
    for ident, namelen in entries:
        parts.append("const struct fsdata_file file__%s[] = { {\n"
                     "%s,\n"
                     "data__%s,\n"
                     "data__%s + %d,\n"
                     "sizeof(data__%s) - %d,\n"
                     "0,\n"
                     "}};\n\n" % (ident, prev, ident, ident, namelen,
                                  ident, namelen))
        prev = "file__" + ident

    parts.append("#define FS_ROOT %s\n" % prev)
    parts.append("#define FS_NUMFILES %d\n" % len(entries))
    open(OUT, "w").write("".join(parts))
    print("wrote %s (%d files)" % (OUT, len(entries)))


if __name__ == "__main__":
    main()
