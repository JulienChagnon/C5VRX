#!/usr/bin/env python3
"""Flash a C5VRX-5 build over USB (C5VRX by Twotoz and the C5VRX contributors).

`idf.py flash` resets the chip through the USB DTR/RTS lines. On a running
C5VRX-5 image that reset lands in a ROM loader that no longer answers (the
modem dump writer is still on), so esptool stops with a pySerial
"Write timeout" and the board stays hung until USB and battery power are
both removed. This runs the route that works instead:

1. the console download key '`' (tools/enter_download.py): the firmware
   stops the writer and Wi-Fi and reboots into the ROM loader itself;
2. esptool --before no-reset write-flash @flash_args, so no line reset
   reaches the chip and no esptool sync is sent to live firmware;
3. --after watchdog-reset: the LP watchdog starts the new image (an RTS
   reset would leave a C5 in the loader).

    idf.py build
    python tools/flash.py COM6
    python tools/flash.py /dev/ttyACM0 --no-key   # board already in the ROM loader
    python tools/flash.py COM6 --app-only         # app image only (flash_app_args)
    python tools/flash.py COM6 --monitor          # then show the console

--app-only skips the bootloader and partition table, so it is faster for
firmware-only changes; use the full set after a partition or bootloader
change, or when switching from another generation.

--monitor replaces `idf.py flash monitor`, whose monitor resets the chip
through the same lines. It reopens the port with DTR and RTS low, prints
the console at 115200 and sends each key as typed (no Enter, as the
receiver expects). Ctrl+] or Ctrl+C exits.

Run it in the ESP-IDF environment, which provides esptool and pyserial.
"""
import argparse
import contextlib
import importlib.util
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_BUILD = HERE.parent / "build"


def esptool_cmd(port, baud, args_file="flash_args"):
    return [sys.executable, "-m", "esptool", "--chip", "esp32c5", "-p", port, "-b", str(baud),
            "--before", "no-reset", "--after", "watchdog-reset", "write-flash", "@" + args_file]


def send_download_key(port):
    sys.path.insert(0, str(HERE))
    from enter_download import enter_download
    return enter_download(port)


def wait_for_port(port, timeout_s=6.0, settle_s=1.0):
    """The ROM loader re-enumerates on USB after the restart; wait until it is back."""
    from serial.tools import list_ports
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if any(p.device.lower() == port.lower() for p in list_ports.comports()):
            break
        time.sleep(0.2)
    time.sleep(settle_s)


def open_quiet(port):
    """Open the console with DTR and RTS low before the open, so it never resets the chip."""
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = port, 115200, 0.05, 0.5
    s.dtr = False
    s.rts = False
    s.open()
    return s


@contextlib.contextmanager
def console_keys():
    """Yield a poll() that returns the next key pressed, without Enter, or None."""
    if os.name == "nt":
        import msvcrt

        def poll():
            if not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
            if ch in "\x00\xe0":   # arrow and function keys: drop the second half
                msvcrt.getwch()
                return None
            return ch
        yield poll
        return
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        yield lambda: None
        return
    import select
    import termios
    import tty
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        yield lambda: os.read(fd, 1).decode(errors="ignore") if select.select([fd], [], [], 0)[0] else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def monitor(port, poll, out=sys.stdout):
    import serial
    print(f"flash: console on {port} at 115200; Ctrl+] or Ctrl+C exits", file=out, flush=True)
    s = None
    try:
        while True:
            if s is None:
                # The watchdog restart re-enumerates USB; keep trying until it is back.
                try:
                    s = open_quiet(port)
                except serial.SerialException:
                    time.sleep(0.2)
                    continue
            try:
                data = s.read(4096)
                if data:
                    out.write(data.decode(errors="replace"))
                    out.flush()
                key = poll()
                if key in ("\x1d", "\x03"):
                    break
                if key:
                    s.write(key.encode("ascii", errors="ignore"))
            except serial.SerialTimeoutException:
                print("\n[flash: console not reading keys]", file=out, flush=True)
            except serial.SerialException:
                with contextlib.suppress(serial.SerialException):
                    s.close()
                s = None
                print("\n[flash: port lost, reopening]", file=out, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if s is not None:
            with contextlib.suppress(serial.SerialException):
                s.close()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Flash C5VRX-5 without a USB line reset.")
    ap.add_argument("port", help="serial port, for example COM6 or /dev/ttyACM0")
    ap.add_argument("--build", type=Path, default=DEFAULT_BUILD,
                    help="idf.py build directory (default: %(default)s)")
    ap.add_argument("-b", "--baud", type=int, default=460800)
    ap.add_argument("--no-key", action="store_true",
                    help="skip the download key: the board is already in the ROM loader "
                         "(BOOT held while plugging in, or an interrupted flash)")
    ap.add_argument("--app-only", action="store_true",
                    help="flash only the app image (flash_app_args), not the bootloader "
                         "and partition table")
    ap.add_argument("--monitor", action="store_true",
                    help="after flashing, show the console without resetting the chip")
    args = ap.parse_args(argv)
    args_file = "flash_app_args" if args.app_only else "flash_args"

    # Check everything before the key: it reboots the receiver into the loader.
    if not (args.build / args_file).is_file():
        print(f"flash: {args.build / args_file} not found; run idf.py build first")
        return 2
    if importlib.util.find_spec("esptool") is None:
        print("flash: esptool not found; run this in the ESP-IDF environment "
              "(export.ps1 / . ./export.sh) or pip install esptool")
        return 2

    if not args.no_key:
        if not send_download_key(args.port):
            print("flash: the firmware did not acknowledge the download key, so nothing was "
                  "flashed (esptool's sync would reach a running receiver).\n"
                  "  Board already in the ROM loader: rerun with --no-key.\n"
                  "  Otherwise remove USB and the battery, hold BOOT while plugging USB "
                  "back in, release it, and rerun with --no-key.")
            return 1
        wait_for_port(args.port)

    rc = subprocess.run(esptool_cmd(args.port, args.baud, args_file), cwd=args.build).returncode
    if rc == 0:
        print("flash: done; the receiver restarts into the new image")
        if args.monitor:
            with console_keys() as poll:
                return monitor(args.port, poll)
    else:
        print("flash: esptool failed. If it timed out, remove USB and the battery, hold BOOT "
              "while plugging USB back in, and rerun with --no-key.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
