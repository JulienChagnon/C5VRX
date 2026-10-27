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
    python tools/flash.py COM6 --monitor-only     # console only, no flashing
    python tools/flash.py COM6 --check            # flash, then a pass/fail health check
    python tools/flash.py COM6 --monitor-only --log bench.log --iq iq/ --iq-every 10

--app-only skips the bootloader and partition table, so it is faster for
firmware-only changes; use the full set after a partition or bootloader
change, or when switching from another generation.

--monitor replaces `idf.py flash monitor`, whose monitor resets the chip
through the same lines. It reopens the port with DTR and RTS low, prints
the console at 115200 and sends each key as typed (no Enter, as the
receiver expects). Ctrl+] or Ctrl+C exits. --monitor-only opens the same
console without flashing (no build or esptool needed).

--log FILE appends every console line with a local timestamp
(HH:MM:SS.mmm), plus the keys sent and the tool's own notes as '#' lines.
tools/field_diag_compare.py reads these logs directly.

--iq DIR saves each IQ snapshot the console prints (key I) as a .bin and
.json pair, in the format of tools/capture_iq_snapshot.py, instead of
32 hex lines on the screen; --iq-every SECONDS also sends I on that period,
for watching the raw IQ while tuning gain, filter or the VTX. A snapshot
is 204.75 us of IQ: a sample of the receiver input, not a range result.

--check waits for the boot (CAL_BOOT), lets the boot phase pass, sends T
and D and exits 1 if the board crashed or rebooted, runs another image
than the one flashed (the FIRMWARE elf_sha256 prefix), reports a chain
fault or a filter that did not read back, or (with --expect-video) has no
video (VIDEO_Q known=0). It ends with one 'CHECK result=pass|fail ...'
line for scripts. With --monitor-only it checks the running board without
waiting for a boot or comparing the image.

The check also reads the heap from T and warns when the largest free block
is below --heap-warn (8192 bytes) or, on firmware that prints heap_min, when
the lowest free heap since boot is below --heap-min-warn (1024 bytes). The
boot peak (menu splash chain plus the receiver tasks) leaves about 2 KB on
a working build, which a reading after the boot cannot show.

Run it in the ESP-IDF environment, which provides esptool and pyserial.
"""
import argparse
import contextlib
import datetime
import hashlib
import importlib.util
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_BUILD = HERE.parent / "build"
sys.path.insert(0, str(HERE))
import capture_iq_snapshot  # noqa: E402

KV = re.compile(r"(\w+)=(\S+)")
BOOT_DONE = ("CAL_BOOT ", "STARTUP done")
BOOT_BANNER = ("rst:0x",)
CRASH = ("Guru Meditation", "ESP_ERROR_CHECK failed", "abort() was called", "task_wdt")
WARN_VERDICTS = ("NOT_CALIBRATED", "DC_CAL_NEEDED", "GAIN_FIXED", "CALIBRATION_OR_PHY sampling phase")


def esptool_cmd(port, baud, args_file="flash_args"):
    return [sys.executable, "-m", "esptool", "--chip", "esp32c5", "-p", port, "-b", str(baud),
            "--before", "no-reset", "--after", "watchdog-reset", "write-flash", "@" + args_file]


def send_download_key(port):
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


def elf_sha256(build):
    """SHA-256 of the built app ELF, as the firmware prints it in FIRMWARE elf_sha256=."""
    with contextlib.suppress(OSError, ValueError, KeyError):
        desc = json.loads((build / "project_description.json").read_text())
        return hashlib.sha256((build / desc["app_elf"]).read_bytes()).hexdigest()
    return None


class Console:
    """Console lines to the screen, the --log file and, with --iq, IQ snapshots to files."""

    def __init__(self, out=None, log=None, iq_dir=None):
        self.out, self.log, self.iq_dir = out or sys.stdout, log, iq_dir
        self.buf, self.shown, self.iq, self.saved = "", 0, None, 0
        self.lines = []   # every complete line

    def feed(self, text):
        self.buf += text
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self.line(line.rstrip("\r"), self.shown)
            self.shown = 0
        self.out.flush()

    def idle(self):
        """Nothing new arrived: show a partial line (a prompt), unless it is IQ data to save."""
        if self.buf[self.shown:] and (self.iq_dir is None or "IQSNAP" not in self.buf):
            self.out.write(self.buf[self.shown:])
            self.out.flush()
            self.shown = len(self.buf)

    def line(self, line, shown=0):
        self.lines.append(line)
        if self.iq_dir is not None and "IQSNAP_" in line and "IQSNAP_REFUSED" not in line:
            self.iq_record(capture_iq_snapshot.record_line(line))
            return
        self.out.write(line[shown:] + "\n")
        self.write_log(line)

    def note(self, text):
        print(text, file=self.out, flush=True)
        self.write_log("# " + " ".join(text.split()))

    def write_log(self, line):
        if self.log:
            self.log.write(f"{datetime.datetime.now():%H:%M:%S.%f}"[:-3] + " " + line + "\n")
            self.log.flush()

    def iq_record(self, rec):
        if rec.startswith("IQSNAP_BEGIN "):
            self.iq = [rec]
        elif self.iq is not None:
            self.iq.append(rec)
        if not rec.startswith("IQSNAP_END"):
            return
        lines, self.iq = self.iq, None
        try:
            if lines is None:
                raise ValueError("no IQSNAP_BEGIN header")
            raw, metadata = capture_iq_snapshot.decode(lines)
        except ValueError as e:
            self.note(f"IQ snapshot not saved: {e}")
            return
        self.saved += 1
        path = self.iq_dir / f"iq_{datetime.datetime.now():%Y%m%d-%H%M%S}_{self.saved:03d}.bin"
        capture_iq_snapshot.save(raw, metadata, path)
        self.note(f"IQ saved {path} gain={metadata.get('gain')} freq_mhz={metadata.get('freq_mhz')}")


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


def monitor(port, poll, console, iq_every=0.0):
    import serial
    console.note(f"flash: console on {port} at 115200; Ctrl+] or Ctrl+C exits")
    s = None
    next_iq = time.monotonic() + iq_every if iq_every else None
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
                    console.feed(data.decode(errors="replace"))
                    # Monitoring can run for days, including large IQ dumps.
                    # Only check_board needs an in-memory transcript.
                    console.lines.clear()
                else:
                    console.idle()
                key = poll()
                if key in ("\x1d", "\x03"):
                    break
                if next_iq is not None and time.monotonic() >= next_iq:
                    next_iq = time.monotonic() + iq_every
                    key = key or "I"
                if key:
                    s.write(key.encode("ascii", errors="ignore"))
                    console.write_log("# key " + key)
            except serial.SerialTimeoutException:
                console.note("\n[flash: console not reading keys]")
            except serial.SerialException:
                with contextlib.suppress(serial.SerialException):
                    s.close()
                s = None
                console.note("\n[flash: port lost, reopening]")
    except KeyboardInterrupt:
        pass
    finally:
        if s is not None:
            with contextlib.suppress(serial.SerialException):
                s.close()
    return 0


def check_board(port, console, booting, boot_timeout=60.0, settle=6.0, status_wait=2.0, diag_wait=5.0):
    """Wait out the boot (if booting), then send T and D; return the console lines read."""
    import serial
    start = len(console.lines)
    s = None

    def pump(seconds, until=None):
        nonlocal s
        mark = len(console.lines)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if s is None:
                try:
                    s = open_quiet(port)
                except serial.SerialException:
                    time.sleep(0.2)
                    continue
            try:
                data = s.read(4096)
            except serial.SerialException:
                with contextlib.suppress(serial.SerialException):
                    s.close()
                s = None
                continue
            if data:
                console.feed(data.decode(errors="replace"))
            if until and any(u in line for line in console.lines[mark:] for u in until):
                return True
        return False

    def send(key):
        nonlocal s
        for _ in range(2):
            if s is None:
                pump(1.0)   # still re-enumerating: give it a moment
            if s is None:
                continue
            try:
                s.write(key)
                console.write_log("# key " + key.decode())
                return
            except serial.SerialException:
                with contextlib.suppress(serial.SerialException):
                    s.close()
                s = None

    try:
        if booting:
            console.note(f"flash: check waits up to {boot_timeout:.0f} s for the boot")
            if pump(boot_timeout, BOOT_DONE):
                pump(settle)   # the boot phase (sampling-phase check) runs 5 s after CAL_BOOT
        send(b"T")
        pump(status_wait)
        send(b"D")
        if pump(diag_wait, ("RANGE_DIAG history",)):
            pump(0.3)
    finally:
        if s is not None:
            with contextlib.suppress(serial.SerialException):
                s.close()
    return console.lines[start:]


def evaluate(lines, booting, elf=None, expect_video=False, heap_warn=8192, heap_min_warn=1024):
    """Judge the check from the console lines: (problems, warnings, summary fields)."""
    problems, warnings, fields = [], [], {}
    boot_at = next((i for i, line in enumerate(lines) if any(b in line for b in BOOT_DONE)), None)
    if booting and boot_at is None:
        problems.append("no CAL_BOOT or STARTUP done line: the receiver did not finish booting")
    for i, line in enumerate(lines):
        if any(c in line for c in CRASH):
            problems.append("crash: " + line.strip())
        elif (not booting or (boot_at is not None and i > boot_at)) and any(b in line for b in BOOT_BANNER):
            problems.append("rebooted after the boot: " + line.strip())

    def last(tag):
        found = [dict(KV.findall(line[line.find(tag):])) for line in lines if tag in line]
        return found[-1] if found else None

    fw = last("FIRMWARE id=")
    if fw is None:
        problems.append("no FIRMWARE line: the board did not answer T")
    else:
        fields["firmware"] = fw.get("id", "?")
        running = fw.get("elf_sha256", "")
        if elf:
            fields["elf_match"] = int(bool(running) and elf.startswith(running))
            if not fields["elf_match"]:
                problems.append(f"running image elf_sha256={running[:16] or '?'} is not the flashed build {elf[:16]}")
    status = last("C5VRX5 version=") or {}
    for key, floor in (("heap_largest", heap_warn), ("heap_min", heap_min_warn)):
        if status.get(key, "").isdigit():   # heap_min only on firmware that prints it
            fields[key] = int(status[key])
            if fields[key] < floor:
                warnings.append(f"{key}={fields[key]} below {floor} bytes (internal heap is nearly exhausted)")
    vq = last("VIDEO_Q ")
    if vq is not None:
        fields.update(video_known=vq.get("known"), snr_x10=vq.get("snr_x10"), lines_x100=vq.get("lines_x100"))
    if expect_video and (vq is None or vq.get("known") != "1"):
        problems.append("no video: VIDEO_Q known=0 (VTX on, same channel, load fitted?)")
    if not any("RANGE_DIAG " in line for line in lines):
        problems.append("no RANGE_DIAG lines: the board did not answer D")
    gain, iq, chain = last("RANGE_DIAG gain "), last("RANGE_DIAG iq "), last("RANGE_DIAG chain ")
    if last("RANGE_DIAG history ") is None or chain is None or "faults" not in chain:
        problems.append("incomplete RANGE_DIAG response: transport health was not confirmed")
    if gain:
        fields["gain"] = gain.get("index")
    if iq:
        fields["cn_est_x10"] = iq.get("cn_est_x10")
    if chain:
        fields["faults"] = chain.get("faults")
        if chain.get("faults", "0") != "0":
            problems.append(f"transport faults={chain['faults']}")
    verdicts = [line.split("verdict=", 1)[1].strip() for line in lines if "RANGE_DIAG verdict=" in line]
    for v in verdicts:
        if v.startswith("CALIBRATION_OR_PHY filter"):
            problems.append(v)
        elif v.startswith(WARN_VERDICTS):
            warnings.append(v)
    fields["verdicts"] = ",".join(v.split()[0] for v in verdicts) or "none"
    # A boot loop repeats the same crash every boot: report each once, with a count.
    counted = {p: problems.count(p) for p in problems}
    problems = [p if n == 1 else f"{p} (x{n})" for p, n in counted.items()]
    return problems, warnings, fields


def run_check(port, console, booting, elf=None, expect_video=False, boot_timeout=60.0,
              heap_warn=8192, heap_min_warn=1024):
    lines = check_board(port, console, booting, boot_timeout)
    problems, warnings, fields = evaluate(lines, booting, elf, expect_video, heap_warn, heap_min_warn)
    for w in warnings:
        console.note("CHECK warn: " + w)
    for p in problems:
        console.note("CHECK fail: " + p)
    console.note(" ".join(["CHECK result=" + ("fail" if problems else "pass")] +
                          [f"{k}={v}" for k, v in fields.items()]))
    return 1 if problems else 0


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
    ap.add_argument("--monitor-only", action="store_true",
                    help="do not flash: only show the console (or --check the running board)")
    ap.add_argument("--log", type=Path, metavar="FILE",
                    help="append the console to FILE with timestamps (field_diag_compare.py reads it)")
    ap.add_argument("--iq", type=Path, metavar="DIR",
                    help="save each IQ snapshot (key I) to DIR as .bin + .json")
    ap.add_argument("--iq-every", type=float, default=0.0, metavar="SECONDS",
                    help="with --iq: send I on this period while monitoring")
    ap.add_argument("--check", action="store_true",
                    help="after flashing, wait for the boot, run T and D and exit 1 if the "
                         "health check fails")
    ap.add_argument("--expect-video", action="store_true",
                    help="with --check: also fail without video (VIDEO_Q known=0)")
    ap.add_argument("--boot-timeout", type=float, default=60.0, metavar="SECONDS",
                    help="with --check: how long to wait for CAL_BOOT (default: %(default)s)")
    ap.add_argument("--heap-warn", type=int, default=8192, metavar="BYTES",
                    help="with --check: warn when T's heap_largest is below this (default: %(default)s)")
    ap.add_argument("--heap-min-warn", type=int, default=1024, metavar="BYTES",
                    help="with --check: warn when T's heap_min (lowest since boot) is below this "
                         "(default: %(default)s)")
    args = ap.parse_args(argv)
    if args.iq_every and not args.iq:
        ap.error("--iq-every needs --iq DIR")
    if args.expect_video and not args.check:
        ap.error("--expect-video needs --check")
    if args.iq_every and not (args.monitor or args.monitor_only):
        ap.error("--iq-every needs --monitor or --monitor-only")
    args_file = "flash_app_args" if args.app_only else "flash_args"

    if not args.monitor_only:
        # Check everything before the key: it reboots the receiver into the loader.
        if not (args.build / args_file).is_file():
            print(f"flash: {args.build / args_file} not found; run idf.py build first")
            return 2
        if importlib.util.find_spec("esptool") is None:
            print("flash: esptool not found; run this in the ESP-IDF environment "
                  "(export.ps1 / . ./export.sh) or pip install esptool")
            return 2
    if args.iq:
        args.iq.mkdir(parents=True, exist_ok=True)

    with contextlib.ExitStack() as stack:
        log = None
        if args.log:
            log = stack.enter_context(open(args.log, "a", encoding="utf-8"))
        console = Console(log=log, iq_dir=args.iq)
        console.write_log(f"# flash.py log {datetime.datetime.now().isoformat(timespec='seconds')} "
                          f"port={args.port} argv={' '.join(sys.argv[1:] if argv is None else argv)}")
        rc = 0
        if not args.monitor_only:
            rc = flash(args, args_file, console)
            if rc != 0:
                return rc
        if args.check:
            elf = None if args.monitor_only else elf_sha256(args.build)
            rc = run_check(args.port, console, not args.monitor_only, elf, args.expect_video, args.boot_timeout,
                           args.heap_warn, args.heap_min_warn)
        if args.monitor or (args.monitor_only and not args.check):
            with console_keys() as poll:
                monitor(args.port, poll, console, args.iq_every)
        return rc


def flash(args, args_file, console):
    if not args.no_key:
        if not send_download_key(args.port):
            console.note("flash: the firmware did not acknowledge the download key, so nothing was "
                         "flashed (esptool's sync would reach a running receiver).\n"
                         "  Board already in the ROM loader: rerun with --no-key.\n"
                         "  Otherwise remove USB and the battery, hold BOOT while plugging USB "
                         "back in, release it, and rerun with --no-key.")
            return 1
        wait_for_port(args.port)

    rc = subprocess.run(esptool_cmd(args.port, args.baud, args_file), cwd=args.build).returncode
    if rc == 0:
        console.note("flash: done; the receiver restarts into the new image")
    else:
        console.note("flash: esptool failed. If it timed out, remove USB and the battery, hold BOOT "
                     "while plugging USB back in, and rerun with --no-key.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
