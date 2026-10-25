import sys
import time
import subprocess
from pathlib import Path
import serial.tools.list_ports

ROOT = Path(__file__).resolve().parent.parent

# Prefer downloaded release artifacts if present
REL = ROOT / "release_download"
if (REL / "c5vrx3.bin").exists():
    bootloader = REL / "bootloader.bin"
    ptable = REL / "partition-table.bin"
    app = REL / "c5vrx3.bin"
    version_txt = (REL / "VERSION").read_text(encoding="utf-8").strip() if (REL / "VERSION").exists() else "release"
else:
    BUILD = ROOT / "build"
    bootloader = BUILD / "bootloader" / "bootloader.bin"
    ptable = BUILD / "partition_table" / "partition-table.bin"
    app = BUILD / "c5vrx3.bin"
    version_txt = "local build"

print("=" * 60)
print(f" AUTO-FLASH WATCHER STARTED (Firmware: {version_txt})")
print(" Waiting for ESP32-C5 to be connected...")
print(" (Flashing starts as soon as the USB connection is detected!)")
print("=" * 60)
sys.stdout.flush()

BT_PORTS = {"COM3", "COM4", "COM5", "COM6", "COM7", "COM8"}

def find_esp_port():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    # Check non-BT ports first
    candidates = [p for p in ports if p not in BT_PORTS]
    if "COM10" in candidates:
        return "COM10"
    if candidates:
        return candidates[0]
    return None

for attempt in range(1800): # 30 minutes
    port = find_esp_port()
    if port:
        print(f"\n[FOUND] Port detected: {port}! Starting flash...")
        sys.stdout.flush()
        cmd = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32c5",
            "-p", port,
            "-b", "460800",
            "--before", "default-reset",
            "--after", "watchdog-reset",
            "write-flash",
            "--flash-mode", "dio",
            "--flash-size", "8MB",
            "--flash-freq", "80m",
            "0x2000", str(bootloader),
            "0x8000", str(ptable),
            "0x10000", str(app),
        ]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print(res.stdout)
        if res.returncode == 0:
            print("\n" + "=" * 60)
            print(f" >>> FLASH COMPLETED ON {port}! <<<")
            print("=" * 60)
            sys.stdout.flush()
            sys.exit(0)
        else:
            print("[RETRY] Flash attempt failed, retrying (hold Boot if needed)...")
            sys.stdout.flush()
    time.sleep(1.0)

print("\nTimeout: No ESP32-C5 port detected.")
sys.exit(1)
