# DoIP/UDS Diagnostic Stack on FreeRTOS (QEMU ARM Cortex-M3)

A bare-metal automotive diagnostic stack implemented in C on FreeRTOS,
running on an emulated ARM Cortex-M3 (LM3S6965) via QEMU. No physical
hardware required.

## What it does

Simulates a real automotive ECU diagnostic session using the DoIP protocol
(ISO 13400) as transport and UDS (ISO 14229) as the diagnostic layer.

Three FreeRTOS tasks communicate via pointer-based queues:

- **doipReceiverTask** — builds DoIP packets with UDS service 0x22
  (ReadDataByIdentifier) and sends them into the stack
- **udsHandlerTask** — validates DoIP protocol version, routes valid
  packets to the UDS layer
- **diagnosticTask** — processes UDS DIDs and returns ECU data:
  - DID 0x0401 → Software version
  - DID 0x0402 → Hardware version

## Architecture


         doipReceiverTask
   DoIPPacket_t* (via FreeRTOS queue)
                    │
                    ▼
udsHandlerTask  ──── validates protocol version
   DoIPPacket_t* (via FreeRTOS queue)
                    │
                    ▼
diagnosticTask  ──── resolves DIDs, outputs response
                    │
                    ▼
UART output (visible in QEMU terminal)

## Build & Run

```bash
git clone https://github.com/FreeRTOS/FreeRTOS.git --recurse-submodules --depth 1
cd FreeRTOS/FreeRTOS/Demo/CORTEX_LM3S6965_GCC_QEMU
cp /path/to/main.c .
cmake -B build -DCMAKE_TOOLCHAIN_FILE=gcc-arm-embedded.cmake
cmake --build build
qemu-system-arm -machine lm3s6965evb -nographic -kernel build/RTOSDemo.elf
```

## Expected output

UDS: sID=0x8001 32
SW Ver:1.0 32
UDS: sID=0x8001 1004
HW Ver:P200C2 1004

## Skills demonstrated

- FreeRTOS task creation, scheduling, and inter-task communication
- Pointer-based FreeRTOS queue design
- DoIP protocol structure (ISO 13400)
- UDS service 0x22 ReadDataByIdentifier (ISO 14229)
- Bare-metal ARM Cortex-M3 embedded C
- QEMU embedded system emulation