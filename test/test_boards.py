"""C5VRX-5 by Twotoz and contributors: board pin maps and images stay in step.

main/board.h (firmware pins), web/boards.js (flasher), the CI board matrix
(.github/workflows/c5vrx5.yml), the sdkconfig overlays and the partition
tables describe the same boards; a board or image added in one place only
fails here. Plain python3 (CI has no PyYAML).
"""
import csv
import json
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
MB = 1024 * 1024
RESERVED = {13, 14} | set(range(16, 23))   # USB; flash/PSRAM bus


def firmware_boards():
    text = (ROOT / "main/board.h").read_text(encoding="utf-8")
    boards = {}
    for name, part, value in re.findall(r"^#define BOARD_([A-Z]+)_(DAC|IQ|BOOT)\s+([0-9, ]+)$", text, re.M):
        boards.setdefault(name.lower(), {})[part.lower()] = [int(v) for v in value.split(",")]
    ids = dict(re.findall(r"CONFIG_C5VRX_BOARD_([A-Z]+)\)\n#define BOARD_ID\s+\"([a-z]+)\"", text))
    ids["XIAO"] = re.search(r"#else\n#define BOARD_ID\s+\"([a-z]+)\"", text).group(1)
    return boards, ids


def web_boards():
    text = (ROOT / "web/boards.js").read_text(encoding="utf-8")
    start = text.index("export const BOARDS = ") + len("export const BOARDS = ")
    return json.loads(text[start:text.index("];", start) + 1])


def ci_matrix():
    text = (ROOT / ".github/workflows/c5vrx5.yml").read_text(encoding="utf-8")
    rows = re.findall(r"- \{ id: ([a-z0-9-]+), flash: ([0-9]+)MB, defaults: \"([^\"]+)\" \}", text)
    return {row[0]: {"flash": int(row[1]), "defaults": row[2].split(";")} for row in rows}


def config(files):
    """Last value of each option across sdkconfig defaults files, in order."""
    values = {}
    for name in files:
        for line in (ROOT / name).read_text(encoding="utf-8").splitlines():
            m = re.match(r"(CONFIG_[A-Z0-9_]+)=(.*)$", line.strip())
            if m:
                values[m.group(1)] = m.group(2).strip('"')
    return values


def built_image(values):
    """Board, flash MB and PSRAM a defaults chain builds."""
    board = next((k[len("CONFIG_C5VRX_BOARD_"):] for k, v in values.items()
                  if k.startswith("CONFIG_C5VRX_BOARD_") and v == "y"), "XIAO")
    # A later overlay that picks another size must also unset the earlier one.
    (flash,) = [int(m.group(1)) for k, v in values.items()
                if (m := re.fullmatch(r"CONFIG_ESPTOOLPY_FLASHSIZE_([0-9]+)MB", k)) and v == "y"]
    return board, flash, values.get("CONFIG_SPIRAM") == "y", values["CONFIG_PARTITION_TABLE_CUSTOM_FILENAME"]


def partition_end(name):
    end = 0
    rows = [r for r in csv.reader((ROOT / name).read_text(encoding="utf-8").splitlines())
            if r and not r[0].lstrip().startswith("#")]
    for row in rows:
        end = max(end, int(row[3].strip(), 16) + int(row[4].strip(), 16))
    return end, {row[0].strip(): int(row[4].strip(), 16) for row in rows}


class BoardsTest(unittest.TestCase):
    def setUp(self):
        self.fw, self.fw_ids = firmware_boards()
        self.web = web_boards()
        self.matrix = ci_matrix()

    def test_firmware_and_flasher_pins_match(self):
        fixed = [b for b in self.web if not b.get("custom")]
        self.assertEqual(sorted(self.fw), sorted(b["id"] for b in fixed))
        self.assertEqual(sorted(self.fw_ids.values()), sorted(self.fw))
        kconfig = (ROOT / "main/Kconfig.projbuild").read_text(encoding="utf-8")
        self.assertEqual(sorted(re.findall(r"config C5VRX_BOARD_([A-Z]+)", kconfig)), sorted(self.fw_ids))
        for board in fixed:
            fw = self.fw[board["id"]]
            self.assertEqual(board["dac"], fw["dac"], board["id"])
            self.assertEqual(board["iq"], fw["iq"], board["id"])
            self.assertEqual([board["boot"]], fw["boot"], board["id"])

    def test_pins_are_usable(self):
        for name, pins in self.fw.items():
            used = pins["dac"] + pins["iq"] + pins["boot"]
            self.assertEqual((len(pins["dac"]), len(pins["iq"])), (6, 8), name)
            self.assertEqual(len(set(used)), len(used), f"{name}: a GPIO is used twice")
            self.assertTrue(all(0 <= p <= 28 for p in used), name)
            self.assertFalse(set(used) & RESERVED, f"{name}: USB or flash/PSRAM pin")

    def test_every_image_is_built_as_described(self):
        images = {}
        for board in self.web:
            for image in board["images"]:
                # A board shares an image only as itself (the same description).
                self.assertEqual(images.setdefault(image["id"], image), image, image["id"])
                if not board.get("custom"):
                    self.assertEqual(image["pins"], board["id"], f"{image['id']} carries another board's pins")
        standard = [i for i in images.values() if i["asset"] == "standard"]
        self.assertEqual([i["id"] for i in standard], ["xiao"])
        self.assertEqual(sorted(self.matrix), sorted(k for k, i in images.items() if i["asset"] != "standard"))
        builds = {"xiao": {"flash": 8, "defaults": ["sdkconfig.defaults"]}, **self.matrix}
        for image_id, build in builds.items():
            image = images[image_id]
            board = next(b for b in self.web if b["id"] == image["pins"])
            if image["asset"] != "standard":
                self.assertEqual(image["asset"], image_id)
            fw_board, flash, psram, table = built_image(config(build["defaults"]))
            self.assertEqual(self.fw_ids[fw_board], image["pins"], image_id)
            self.assertEqual(flash, image["flashMb"], image_id)
            self.assertEqual(flash, build["flash"], image_id)
            self.assertEqual(psram, image["psram"], image_id)
            end, _ = partition_end(table)
            self.assertLessEqual(end, flash * MB, f"{image_id}: {table} does not fit")
            if psram:
                self.assertNotIn(15, board["dac"] + board["iq"], f"{image_id}: GPIO15 is the PSRAM chip select on modules")

    def test_layouts_show_every_pin(self):
        for board in self.web:
            for layout in board["layouts"]:
                where = f"{board['id']} {layout['id']}"
                header = [g for g, _ in layout["left"] + layout["right"] if g is not None]
                self.assertEqual(len(set(header)), len(header), f"{where}: a GPIO on two header pins")
                self.assertTrue(all(0 <= g <= 28 for g in header + layout["internal"]), where)
                self.assertFalse(set(header) & set(layout["internal"]), f"{where}: on a header and inside")
                for p in board["dac"]:
                    self.assertIn(p, header, f"{where}: DAC GPIO{p} is on no header")
                for p in board["iq"] + [board["boot"]]:
                    self.assertIn(p, header + layout["internal"], f"{where}: GPIO{p} is missing from the picture")
                for p in board["dac"]:
                    self.assertNotIn(str(p), board["parts"], f"{where}: DAC GPIO{p} has an onboard part")

    def test_flasher_and_firmware_share_the_dac_and_the_pin_table(self):
        dac = (ROOT / "main/dac_check.h").read_text(encoding="utf-8")
        ohms = [int(v) for v in re.search(r"dac_check_ohms\[DAC_CHECK_BITS\] = \{([^}]*)\}", dac).group(1).replace("u", "").split(",")]
        js = (ROOT / "web/boards.js").read_text(encoding="utf-8")
        self.assertEqual(ohms, [int(v) for v in re.search(r"DAC_OHMS = \[([^\]]*)\]", js).group(1).split(",")])
        load = re.search(r"DAC_CHECK_LOAD_OHMS\s+(\d+)", dac).group(1)
        self.assertIn("DAC_LOAD_OHMS = " + load + ";", js)
        board_h = (ROOT / "main/board.h").read_text(encoding="utf-8")
        struct = re.search(r"typedef struct \{(.*?)\} board_pins_t;", board_h, re.S).group(1)
        fields = re.findall(r"(\w+) (\w+)(?:\[(\d+)\])?;", struct)
        self.assertEqual([(n, int(c or 1)) for _, n, c in fields],
                         [("magic", 8), ("dac", 6), ("iq", 8), ("boot", 1), ("custom", 1), ("reserved", 8)])
        magic = "".join(re.findall(r"'(.)'", re.search(r"#define BOARD_PINS_MAGIC\s+\{([^}]*)\}", board_h).group(1)))
        image_js = (ROOT / "web/image-pins.js").read_text(encoding="utf-8")
        self.assertEqual(magic, bytes(int(v, 16) for v in re.search(r"PIN_MAGIC = \[([^\]]*)\]", image_js).group(1).split(",")).decode())
        for offset in ("at + 8, at + 14", "at + 14, at + 22", "bytes[at + 22]", "bytes[at + 23]"):
            self.assertIn(offset, image_js)

    def test_partition_tables_keep_the_flight_log(self):
        sizes = {name.name: partition_end(name.name)[1] for name in ROOT.glob("partitions*.csv")}
        self.assertIn("partitions_4mb.csv", sizes)
        for name, parts in sizes.items():
            self.assertEqual(parts["flightlog"], sizes["partitions.csv"]["flightlog"], name)
            self.assertGreaterEqual(parts["factory"], 2 * MB, name)


if __name__ == "__main__":
    unittest.main()
