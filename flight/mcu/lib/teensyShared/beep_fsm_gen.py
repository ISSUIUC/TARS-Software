import json
import argparse
from pathlib import Path


class Sequence:
    def __init__(self, name: str, sequence: list[list]):
        self.name = name

        self.sequence: list[tuple[int | None, int]] = []

        for pitch, length in sequence:
            if pitch is None:
                self.sequence.append((None, int(length * 1000)))
            else:
                self.sequence.append((pitch, int(length * 1000)))
    
    def num_entries(self) -> int:
        return len(self.sequence)

    def generate_enum_entries(self) -> list[str]:
        return [f"{self.name}_{i}" for i in range(len(self.sequence))]

    def generate_table_entries(self) -> list[str]:
        return [f"/* {self.name} {i} */ {{ {pitch if pitch else 0}, {length}, {self.name}_{(i+1) % len(self.sequence)} }}" for i, (pitch, length) in enumerate(self.sequence)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path)

    source_file = parser.parse_args().file
    data = json.loads(source_file.read_text())

    sequence_infos: list[Sequence] = []

    for sequence_info in data:
        sequence_infos.append(Sequence(sequence_info["name"], sequence_info["sequence"]))

    generated = ""
    generated += "enum BuzzerState {\n"
    generated += "    INITIAL,\n"
    generated += "\n".join("    " + entry + "," for info in sequence_infos for entry in info.generate_enum_entries())
    generated += "\n};\n\n"

    generated += f"unsigned int BuzzerStates[{sum(info.num_entries() for info in sequence_infos) + 1}][3] = {{\n"
    generated += "    /* INITIAL*/ {0, 0, INITIAL},\n"
    generated += "\n".join("    " + entry + "," for info in sequence_infos for entry in info.generate_table_entries())
    generated += "\n};"

    print(generated)


if __name__ == "__main__":
    main()
