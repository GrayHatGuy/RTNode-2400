#!/usr/bin/env python3

# Copyright (C) 2024, Mark Qvist

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import json
import hashlib

major_version = None
minor_version = None
target_version = None

file = open("Config.h", "rb")
config_data = file.read().splitlines()
for line in config_data:
    dline = line.decode("utf-8").strip()
    components = dline.split()
    if dline.startswith("#define MAJ_VERS"):
        major_version = "%01d" % ord(bytes.fromhex(dline.split()[2].split("x")[1]))
    if dline.startswith("#define MIN_VERS"):
        minor_version = "%02d" % ord(bytes.fromhex(dline.split()[2].split("x")[1]))

target_version = major_version+"."+minor_version

release_hashes = {}

# Scan PlatformIO build output directories for merged firmware binaries.
# The release archive (rtnode_firmware.zip) is built from these merged files.
pio_build_dir = ".pio/build"
if os.path.isdir(pio_build_dir):
    for env_name in sorted(os.listdir(pio_build_dir)):
        env_dir = os.path.join(pio_build_dir, env_name)
        if not os.path.isdir(env_dir):
            continue
        for filename in sorted(os.listdir(env_dir)):
            if filename.startswith("rnode_firmware") and filename.endswith(".bin"):
                filepath = os.path.join(env_dir, filename)
                if not os.path.isfile(filepath):
                    continue
                with open(filepath, "rb") as file:
                    release_hashes[filename] = {
                        "hash": hashlib.sha256(file.read()).hexdigest(),
                        "version": target_version,
                        "env": env_name,
                    }

print(json.dumps(release_hashes))
