#!/usr/bin/env python3
import sys
import zlib
import hashlib
import struct

if len(sys.argv) != 2:
    print("Usage: python update_boot_header.py <app_bin>")
    sys.exit(1)

app_bin_file = sys.argv[1]

# Read existing binary
with open(app_bin_file, "rb") as f:
    app_data = f.read()

app_size = len(app_data) + 0x1000  # Account for header size
crc32 = zlib.crc32(app_data) & 0xFFFFFFFF
sha256 = hashlib.sha256(app_data).digest()

# Convert SHA-256 to 8x uint32 array (big-endian)
hash_words = struct.unpack(">8I", sha256)

# Create boot header (56 bytes)
boot_header_bin = struct.pack(
    ">6I8I",
    0xB00710AD,             # magic
    app_size,               # img_size
    crc32,                  # crc32
    1,                      # version
    0x08010000,             # load_addr
    0x08010000 + 0x1000,    # entry (after header)
    *hash_words
)

# Pad header to 128 bytes
boot_header_bin = boot_header_bin.ljust(0x1000, b'\x00')

# Prepend header safely
new_bin = boot_header_bin + app_data

# Overwrite the original file
with open(app_bin_file, "wb") as f:
    f.write(new_bin)

print(f"[boot_header] Prepended boot header to {app_bin_file}")
print(f"Original app size: 0x{app_size:X} bytes, CRC32: 0x{crc32:08X}")
