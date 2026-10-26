"""C5VRX-5 by Twotoz and contributors: tools/flash.py never line-resets or syncs a running receiver."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import serial as flash_serial

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

    def monitor(self, port, keys):
        keys = list(keys)
        out = io.StringIO()
        with mock.patch.object(flash_serial, "Serial", port), mock.patch.object(flash.time, "sleep"):
            flash.monitor("COM6", lambda: keys.pop(0) if keys else "\x1d", out)
        return out.getvalue()

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


if __name__ == "__main__":
    unittest.main()
