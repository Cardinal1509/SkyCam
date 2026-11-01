import sys
from esptool.cmds import detect_chip, run_stub, write_flash

PORT = "COM5"
BAUD = 230400

print(f"Connecting to ESP32 on {PORT}...")
esp = detect_chip(PORT, 115200, connect_mode="default-reset")
print(f"Connected: {esp.CHIP_NAME}")

# Set 512 byte block size to prevent FTDI FIFO overflow
esp.ESP_RAM_BLOCK = 512

print("Uploading and launching RAM stub flasher...")
esp = run_stub(esp)
print("Stub flasher active!")

if BAUD != 115200:
    print(f"Switching baud rate to {BAUD}...")
    esp.change_baud(BAUD)

files = [
    (0x1000, r"C:\Users\judel\AppData\Local\arduino\sketches\86334C912F441A5528E0A270BD75581E\cam_program.ino.bootloader.bin"),
    (0x8000, r"C:\Users\judel\AppData\Local\arduino\sketches\86334C912F441A5528E0A270BD75581E\cam_program.ino.partitions.bin"),
    (0xe000, r"C:\Users\judel\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.12\tools\partitions\boot_app0.bin"),
    (0x10000, r"C:\Users\judel\AppData\Local\arduino\sketches\86334C912F441A5528E0A270BD75581E\cam_program.ino.bin"),
]

print("Writing flash with compression and hash verification...")
write_flash(
    esp,
    files,
    flash_freq="40m",
    flash_mode="dio",
    flash_size="4MB",
    compress=True,
    verify=True,
)

print("Flash complete! Hard resetting ESP32...")
esp.hard_reset()
print("Success!")
