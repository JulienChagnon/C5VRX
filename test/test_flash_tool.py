"""C5VRX-5 by Twotoz and contributors: tools/flash.py never line-resets or syncs a running receiver."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

# CI runs this with the container's plain python3, which has no pyserial, so
# the monitor tests swap in this stand-in for the serial module.
flash_serial = types.ModuleType("serial")
flash_serial.SerialException = type("SerialException", (OSError,), {})
flash_serial.SerialTimeoutException = type("SerialTimeoutException", (flash_serial.SerialException,), {})

spec = importlib.util.spec_from_file_location("flash", Path(__file__).parents[1] / "tools/flash.py")
flash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flash)


def run(argv, key_ack=True, has_esptool=True, flash_args=("flash_args",), rc=0):
    """main() with the board side mocked; returns (rc, key mock, esptool mock, output)."""
    with tempfile.TemporaryDirectory() as build:
        for name in flash_args:
            (Path(build) / name).write_text("--flash-mode dio\n0x10000 c5vrx.bin\n")
        key = mock.Mock(return_value=key_ack)
        esptool = mock.Mock(return_value=mock.Mock(returncode=rc))
        out = io.StringIO()
        with mock.patch.object(flash, "send_download_key", key), \
                mock.patch.object(flash, "wait_for_port"), \
                mock.patch.object(flash.subprocess, "run", esptool), \
                mock.patch.object(flash.importlib.util, "find_spec",
                                  return_value=object() if has_esptool else None), \
                contextlib.redirect_stdout(out):
            code = flash.main(argv + ["--build", build])
        return code, key, esptool, out.getvalue(), build


class FlashToolTest(unittest.TestCase):
    def test_command_has_no_line_reset(self):
        cmd = flash.esptool_cmd("COM6", 460800)
        self.assertEqual(cmd[cmd.index("--before") + 1], "no-reset")
        self.assertEqual(cmd[cmd.index("--after") + 1], "watchdog-reset")
        self.assertEqual(cmd[cmd.index("--chip") + 1], "esp32c5")
        self.assertEqual(cmd[-2:], ["write-flash", "@flash_args"])

    def test_key_then_flash_from_build_dir(self):
        code, key, esptool, _, build = run(["COM6"])
        self.assertEqual(code, 0)
        key.assert_called_once_with("COM6")
        self.assertEqual(esptool.call_args.kwargs["cwd"], Path(build))

    def test_no_ack_flashes_nothing(self):
        code, _, esptool, out, _ = run(["COM6"], key_ack=False)
        self.assertEqual(code, 1)
        esptool.assert_not_called()
        self.assertIn("--no-key", out)

    def test_no_key_skips_the_key(self):
        code, key, esptool, _, _ = run(["COM6", "--no-key"])
        self.assertEqual(code, 0)
        key.assert_not_called()
        esptool.assert_called_once()

    def test_checks_run_before_the_key(self):
        # The key reboots the receiver into the loader, so a missing build or
        # esptool must stop the tool before it is sent.
        for kw in ({"flash_args": ()}, {"has_esptool": False}):
            code, key, esptool, _, _ = run(["COM6"], **kw)
            self.assertEqual(code, 2)
            key.assert_not_called()
            esptool.assert_not_called()

    def test_app_only_uses_flash_app_args(self):
        code, _, esptool, _, _ = run(["COM6", "--app-only"], flash_args=("flash_args", "flash_app_args"))
        self.assertEqual(code, 0)
        self.assertEqual(esptool.call_args.args[0][-1], "@flash_app_args")
        # Only the full set present: --app-only must not fall back to it.
        code, key, _, out, _ = run(["COM6", "--app-only"])
        self.assertEqual(code, 2)
        key.assert_not_called()
        self.assertIn("flash_app_args", out)

    def test_esptool_failure_is_returned(self):
        code, _, _, out, _ = run(["COM6"], rc=2)
        self.assertEqual(code, 2)
        self.assertIn("--no-key", out)

    def test_monitor_only_after_a_good_flash(self):
        for rc, calls in ((0, 1), (2, 0)):
            mon = mock.Mock(return_value=0)
            with mock.patch.object(flash, "monitor", mon), \
                    mock.patch.object(flash, "console_keys", lambda: contextlib.nullcontext(None)):
                code, *_ = run(["COM6", "--monitor"], rc=rc)
            self.assertEqual(code, rc)
            self.assertEqual(mon.call_count, calls)

    def test_monitor_only_needs_no_build_and_flashes_nothing(self):
        mon = mock.Mock(return_value=0)
        with mock.patch.object(flash, "monitor", mon), \
                mock.patch.object(flash, "console_keys", lambda: contextlib.nullcontext(None)):
            code, key, esptool, _, _ = run(["COM6", "--monitor-only"], flash_args=(), has_esptool=False)
        self.assertEqual(code, 0)
        key.assert_not_called()
        esptool.assert_not_called()
        mon.assert_called_once()

    def test_check_after_flash_and_on_a_running_board(self):
        for argv, flashes, booting in ((["--check"], 1, True), (["--monitor-only", "--check"], 0, False)):
            chk = mock.Mock(return_value=1)
            mon = mock.Mock()
            with mock.patch.object(flash, "run_check", chk), mock.patch.object(flash, "monitor", mon):
                code, _, esptool, _, _ = run(["COM6"] + argv)
            self.assertEqual(code, 1)
            self.assertEqual(esptool.call_count, flashes)
            self.assertEqual(chk.call_args.args[2], booting)
            mon.assert_not_called()   # --check alone exits with its result

    def test_option_dependencies(self):
        for argv in (["--iq-every", "5", "--monitor"], ["--expect-video"],
                     ["--iq", "x", "--iq-every", "5"]):
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                flash.main(["COM6"] + argv)


class FakePort:
    """Serial port double: records line states at open, replays reads, can fail."""
    opened = []

    def __init__(self, reads=(), fail_open=0):
        self.reads, self.fail_open, self.writes = list(reads), fail_open, []

    def __call__(self):
        return self

    def open(self):
        if self.fail_open:
            self.fail_open -= 1
            raise flash_serial.SerialException("gone")
        FakePort.opened.append((self.dtr, self.rts))

    def read(self, n):
        item = self.reads.pop(0) if self.reads else b""
        if isinstance(item, Exception):
            raise item
        return item

    def write(self, b):
        self.writes.append(b)

    def close(self):
        pass



class MonitorTest(unittest.TestCase):
    def setUp(self):
        FakePort.opened = []

    def monitor(self, port, keys, console=None, iq_every=0.0):
        keys = list(keys)
        console = console or flash.Console(io.StringIO())
        flash_serial.Serial = port
        with mock.patch.dict(sys.modules, {"serial": flash_serial}), mock.patch.object(flash.time, "sleep"):
            flash.monitor("COM6", lambda: keys.pop(0) if keys else "\x1d", console, iq_every)
        return console.out.getvalue()

    def test_opens_with_lines_low_and_forwards_keys(self):
        port = FakePort([b"boot\n", b"", b"T: ch A1\n"])
        out = self.monitor(port, [None, "T", None])
        self.assertEqual(FakePort.opened, [(False, False)])
        self.assertIn("boot\nT: ch A1\n", out)
        self.assertEqual(port.writes, [b"T"])

    def test_exit_keys_are_not_sent(self):
        for key in ("\x1d", "\x03"):
            port = FakePort()
            self.monitor(port, [key, "T"])
            self.assertEqual(port.writes, [])

    def test_reopens_after_reenumeration(self):
        # Open fails while the restart re-enumerates USB, then the port drops mid-read.
        port = FakePort([b"a", flash_serial.SerialException("lost"), b"b"], fail_open=3)
        out = self.monitor(port, [None, None, None])
        self.assertEqual(FakePort.opened, [(False, False), (False, False)])
        self.assertIn("port lost, reopening", out)
        self.assertIn("b", out.split("reopening")[1])


    def test_iq_every_sends_i_on_the_period(self):
        clock = iter(range(0, 1000, 4))   # each monotonic() call is 4 s later
        port = FakePort()
        with mock.patch.object(flash.time, "monotonic", lambda: next(clock)):
            self.monitor(port, [None, None, None, "T", None], iq_every=10)
        self.assertIn(b"I", port.writes)
        self.assertIn(b"T", port.writes)
        self.assertLess(port.writes.count(b"I"), 4)


IQ = bytes(range(256)) * 31 + bytes(254)


def snapshot_lines(data=IQ, corrupt=False):
    """The firmware's I output: header, 256-byte hex lines, checksum."""
    h = flash.capture_iq_snapshot.fnv1a(data)
    lines = [f"IQSNAP_BEGIN bytes=8190 rate_hz=40000000 format=Ihigh_Qlow gain=60 lane=2 freq_mhz=5800 "
             f"fnv1a={h:08x} fw=abc"]
    for off in range(0, len(data), 256):
        lines.append(f"IQSNAP_DATA offset={off} {data[off:off + 256].hex()}")
    lines.append(f"IQSNAP_END fnv1a={(h ^ corrupt):08x}")
    return lines


class ConsoleTest(unittest.TestCase):
    def test_log_is_timestamped_and_field_diag_compare_reads_it(self):
        spec = importlib.util.spec_from_file_location("compare", Path(__file__).parents[1] / "tools/field_diag_compare.py")
        compare = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(compare)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "bench.log"
            with open(path, "w") as log:
                c = flash.Console(io.StringIO(), log)
                c.feed("FIELD_DIAG t=1 sync_valid_pct=100 gain=70 max=83\r\nFIELD_DIAG t=2 sync_")
                c.feed("valid_pct=0 gain=70 max=83\n")
                c.note("CHECK result=pass")
            text = path.read_text()
            self.assertRegex(text.splitlines()[0], r"^\d\d:\d\d:\d\d\.\d{3} FIELD_DIAG t=1 ")
            self.assertIn(" # CHECK result=pass", text)
            self.assertEqual(compare.summary(str(path))["seconds"], 2)

    def test_partial_line_shown_once(self):
        c = flash.Console(io.StringIO())
        c.feed("menu> ")
        c.idle()
        c.feed("ok\n")
        self.assertEqual(c.out.getvalue(), "menu> ok\n")
        self.assertEqual(c.lines, ["menu> ok"])

    def test_iq_snapshot_saved_not_printed(self):
        with tempfile.TemporaryDirectory() as d:
            c = flash.Console(io.StringIO(), iq_dir=Path(d))
            # A STATUS prefix joined to the header, and a stream line in the middle.
            lines = snapshot_lines()
            lines[0] = "STATUS x " + lines[0]
            lines.insert(5, "FIELD_DIAG t=9 gain=60")
            c.feed("\n".join(lines) + "\n")
            bins = list(Path(d).glob("iq_*.bin"))
            self.assertEqual(len(bins), 1)
            self.assertEqual(bins[0].read_bytes(), IQ)
            self.assertIn('"freq_mhz": "5800"', bins[0].with_suffix(".json").read_text())
            out = c.out.getvalue()
            self.assertNotIn("IQSNAP_DATA", out)
            self.assertIn("FIELD_DIAG t=9", out)
            self.assertIn("IQ saved", out)

    def test_bad_iq_snapshot_reported(self):
        with tempfile.TemporaryDirectory() as d:
            c = flash.Console(io.StringIO(), iq_dir=Path(d))
            c.feed("\n".join(snapshot_lines(corrupt=True)) + "\nIQSNAP_REFUSED context_busy\n")
            self.assertEqual(list(Path(d).iterdir()), [])
            self.assertIn("not saved: IQ checksum mismatch", c.out.getvalue())
            self.assertIn("IQSNAP_REFUSED context_busy", c.out.getvalue())

    def test_without_iq_dir_snapshots_stay_on_screen(self):
        c = flash.Console(io.StringIO())
        c.feed("\n".join(snapshot_lines()) + "\n")
        self.assertIn("IQSNAP_DATA offset=0 ", c.out.getvalue())


ELF = "ab" * 32
GOOD = [
    "ESP-ROM:esp32c5-eco2-20250121",
    "rst:0x1 (POWERON),boot:0x18 (SPI_FAST_FLASH_BOOT)",
    "CAL_BOOT mode=baseline ms=410 searches=0 sweeps=0 dc_trim=0 boot_phase_budget_ms=600",
    "BOOT_PHASE ms=320 budget_ms=600 state=settled",
    f'FIRMWARE id=c5v5-1 version=1 commit=abc elf_sha256={ELF[:9]} built="Oct 26 2026" idf=v6.0.2 cal_dump=1',
    "VIDEO_Q known=1 snr_x10=312 lines_x100=9850 lines=15 windows=60 periods=4 (end-to-end)",
    "RANGE_DIAG gain index=54 max=83 rf_stage=3 rf_code=1 bb=2 fine=0 top_rf_stage_from=70 at_max=0 state=2",
    "RANGE_DIAG chain sphase=settled dc_ok=1 dc_held=1 lane=ultrafine faults=0",
    "RANGE_DIAG iq p50=9 coherence=80 cn_est_x10=180 (model, 12 MHz class) origin_pm=10 clip_pm=0 video_recent=1",
    "RANGE_DIAG verdict=VIDEO_EVIDENCE strong C/N estimate with gain headroom",
    "RANGE_DIAG history seconds=60 sync_evidence_ok_pct=100 at_max_gain_pct=0 dropouts=0",
]


class EvaluateTest(unittest.TestCase):
    def test_healthy_board_passes(self):
        problems, warnings, fields = flash.evaluate(GOOD, True, ELF, expect_video=True)
        self.assertEqual((problems, warnings), ([], []))
        self.assertEqual(fields["elf_match"], 1)
        self.assertEqual((fields["snr_x10"], fields["gain"], fields["cn_est_x10"]), ("312", "54", "180"))
        self.assertEqual(fields["verdicts"], "VIDEO_EVIDENCE")

    def fails(self, lines, booting=True, elf=ELF, **kw):
        return " | ".join(flash.evaluate(lines, booting, elf, **kw)[0])

    def swap(self, prefix, new):
        return [new if line.startswith(prefix) else line for line in GOOD]

    def test_failures(self):
        self.assertIn("not the flashed build", self.fails(GOOD, elf="cd" * 32))
        self.assertIn("did not finish booting", self.fails(GOOD[4:]))
        self.assertIn("rebooted after the boot", self.fails(GOOD + ["rst:0xc (SW_CPU),boot:0x18"]))
        self.assertIn("crash", self.fails(GOOD + ["Guru Meditation Error: Core 0 panic'ed"]))
        self.assertIn("faults=3", self.fails(self.swap("RANGE_DIAG chain", "RANGE_DIAG chain sphase=settled faults=3")))
        self.assertIn("did not answer T", self.fails([line for line in GOOD if "FIRMWARE" not in line]))
        self.assertIn("did not answer D", self.fails([line for line in GOOD if "RANGE_DIAG" not in line]))
        self.assertIn("filter not read back", self.fails(GOOD + [
            "RANGE_DIAG verdict=CALIBRATION_OR_PHY filter not read back as requested"]))
        no_video = self.swap("VIDEO_Q", "VIDEO_Q known=0 snr_x10=0 lines_x100=0")
        self.assertIn("no video", self.fails(no_video, expect_video=True))
        self.assertEqual(self.fails(no_video), "")

    def test_running_board_check_skips_boot_and_image(self):
        self.assertEqual(self.fails(GOOD[4:], booting=False, elf=None), "")

    def test_boot_loop_reported_once_with_a_count(self):
        # Board, 2026-10-26: a stale sdkconfig ran out of heap in receiver_start every boot.
        crash = ["ESP_ERROR_CHECK failed: esp_err_t 0x101 (ESP_ERR_NO_MEM) at 0x420182f6",
                 "abort() was called at PC 0x4080c967 on core 0", "Rebooting...",
                 "ESP-ROM:esp32c5-eco2-20250121", "rst:0xc (SW_CPU),boot:0x18 (SPI_FAST_FLASH_BOOT)",
                 "CAL_BOOT mode=baseline ms=0"]
        problems = flash.evaluate(GOOD[:2] + GOOD[2:3] + crash * 3 + GOOD[3:], True, ELF)[0]
        self.assertEqual(len(problems), 3)
        self.assertIn("ESP_ERR_NO_MEM", problems[0])
        self.assertTrue(all(p.endswith("(x3)") for p in problems))

    def test_settings_left_behind_only_warn(self):
        problems, warnings, _ = flash.evaluate(GOOD + [
            "RANGE_DIAG verdict=GAIN_FIXED a fixed gain is saved across boots",
            "RANGE_DIAG verdict=CALIBRATION_OR_PHY sampling phase unverified (manual CALIBRATE only)"], True, ELF)
        self.assertEqual(problems, [])
        self.assertEqual(len(warnings), 2)


class CheckBoardTest(unittest.TestCase):
    def test_waits_for_boot_then_sends_t_and_d(self):
        FakePort.opened = []
        text = "\n".join(GOOD) + "\n"
        port = FakePort([b"", text[:150].encode(), b"", text[150:].encode()], fail_open=2)
        flash_serial.Serial = port
        clock = (x * 0.1 for x in range(100000))
        console = flash.Console(io.StringIO())
        with mock.patch.dict(sys.modules, {"serial": flash_serial}), mock.patch.object(flash.time, "sleep"), \
                mock.patch.object(flash.time, "monotonic", lambda: next(clock)):
            lines = flash.check_board("COM6", console, True)
        self.assertEqual(port.writes, [b"T", b"D"])
        self.assertTrue(all(o == (False, False) for o in FakePort.opened))
        self.assertEqual(lines, GOOD)


if __name__ == "__main__":
    unittest.main()
