from __future__ import annotations

import json
import argparse
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Item:
    sequence: Sequence
    index: int
    pitch: int | None
    length: int
    next_item: Item | None

    @property
    def enum_entry(self) -> str:
        return f"{self.sequence.name}__STATE_{self.index}"

    @property
    def table_entry(self) -> str:
        if self.pitch is None:
            pitch = "0"
        else:
            pitch = str(self.pitch)

        if self.next_item is None:
            next_item = "INITIAL"
        else:
            next_item = self.next_item.enum_entry

        return f"{{ {pitch}, {self.length}, {next_item} }}"


class Sequence:
    def __init__(self, name: str):
        self.name = name
        self._items: list[Item] = []

    @classmethod
    def from_json(cls, json_structure) -> Sequence:
        name = json_structure["name"]
        obj = cls(name)

        for item in json_structure["sequence"]:
            if item is not None:
                pitch, length = item
                obj._add_item(pitch, int(length * 1000))
            else:
                break
        else:
            # in python, an "else" branch on an "if" is only called whenever the loop DID NOT exit with a break
            # thus, this code only runs if the sequence doesn't have a "null" in it
            obj._create_loop()

        return obj

    def _add_item(self, pitch: int | None, length: int):
        item = Item(self, len(self._items), pitch, length, None)
        if self._items:
            self._items[-1].next_item = item
        self._items.append(item)

    def _create_loop(self):
        self._items[-1].next_item = self._items[0]

    def num_entries(self) -> int:
        return len(self._items)

    def generate_enum_entries(self) -> list[str]:
        return [item.enum_entry for item in self._items]

    def generate_table_entries(self) -> list[str]:
        return [f"/* {item.enum_entry} */ {item.table_entry}" for item in self._items]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path)

    source_file = parser.parse_args().file
    data = json.loads(source_file.read_text())

    sequence_infos: list[Sequence] = []

    for sequence_info in data:
        sequence_infos.append(Sequence.from_json(sequence_info))

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
