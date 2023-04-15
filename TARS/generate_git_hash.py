# Script to generate `hash.h` file containing the latest commit hash that modified packet.h
#
# Autogenerates src/mcu_main/gen/git_hash.c

import os, subprocess

PACKET_H_PATH = "src/common/packet.h"
FILE_PATH = "src/mcu_main/gen/git_hash.c"


# Get the git hash  of the latest commit that modified packet.h
git_command = ["git", "log", "-n", "1", "--pretty=format:%H", "--", PACKET_H_PATH]
git_hash = subprocess.run(git_command, stdout=subprocess.PIPE).stdout.decode('utf-8')
git_hash = git_hash[:-1]  # get rid of trailing new line


# note: git hashes are always 40 characters
file_contents = f"""
/*
 * Auto generated file containing the git hash of the latest commit that modified packet.h.
 *
 * See generate_git_hash.py
 */
char PACKET_H_GIT_HASH[41] = "{git_hash}";
"""

os.makedirs(os.path.dirname(FILE_PATH), exist_ok=True)
with open(FILE_PATH, "w") as f:
        f.write(file_contents)

print(f"Wrote hash {git_hash} to file {FILE_PATH}")