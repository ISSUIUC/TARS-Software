import subprocess
import argparse
import pathlib
from typing import IO

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=pathlib.Path)
    parser.add_argument("-p", "--packet", type=pathlib.Path, default=None)
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("-e", "--emit", choices=["csv"], default="csv")
    parser.add_argument("-o", "--out", type=pathlib.Path, default=None)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.packet is None:
        arguments.packet = pathlib.Path(__file__).parent.parent / "src" / "common" / "packet.h"
        if arguments.verbose:
            print(f"Note: No packet.h file was specified, defaulting to {arguments.packet}")

    if arguments.out is None:
        arguments.out = arguments.file.with_suffix(".csv")   # fix this if we support json at some point
        if arguments.verbose:
            print(f"Note: No output file was specified, defaulting to {arguments.out}")

    commit = subprocess.run(["git", "log", "-n", "1", "--pretty=format:%H", "--", arguments.packet], stdout=subprocess.PIPE).stdout.decode('utf-8')
    if arguments.verbose:
        print(f"Note: packet.h has version {commit}")

    with arguments.file.open("rb") as data_file:
        data_file: IO

        raw_commit = data_file.read(41)[:40]
        data_file_commit = raw_commit.decode("utf8")
        if data_file_commit != commit:
            print(f"Commits do not match (packet.h: {commit[:6]}, {arguments.file.name}: {data_file_commit[:6]})")
            if arguments.verbose:
                print(f"Note: {arguments.file.name} has raw version 0x{raw_commit.hex()}")
                print(f"Note: This decodes to {data_file_commit}")
            # return

    cpp_parser = pathlib.Path(__file__).parent / "data_parser.cpp"
    cpp_compiled_parser = pathlib.Path(__file__).parent / "data_parser"

    if not cpp_compiled_parser.exists():
        subprocess.run(["g++", cpp_parser, "-o", cpp_compiled_parser, "-O0", f"-DPACKET_INCLUDE_PATH=\"{arguments.packet}\""])

    csv_data = subprocess.run([cpp_compiled_parser, arguments.file], stdout=subprocess.PIPE).stdout.decode('utf8', errors="ignore")
    csv_data = csv_data.replace("\r\n", "\n")
    with arguments.out.open("w") as output_file:
        output_file.write(csv_data)


if __name__ == '__main__':
    main()
