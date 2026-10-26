"""C5VRX-5 by Twotoz and contributors: tools/flash.py never line-resets or syncs a running receiver."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest import mock

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


if __name__ == "__main__":
    unittest.main()
