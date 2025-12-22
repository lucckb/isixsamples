# ISIX-RTOS v3 mini operating system for Cortex-M0 / M3 / M4 / M7 Samples repository
More information at [blog post](https://www.emsyslabs.com/isix-rtos-v3-mini-operating-system-for-cortex-m0-m3-m4-m7-functional-description-and-system-characteristics/)

## Building with Meson

This repository has been migrated from WAF to Meson build system. The `isixrtos` submodule uses Meson for building.

### Prerequisites

- Meson >= 1.2
- ARM GCC toolchain (arm-none-eabi-gcc)
- Python 3

### Building Examples

**IMPORTANT**: Meson does not support including `.ini` files within other `.ini` files. You must provide all cross files separately using multiple `--cross-file` arguments.

#### STM32F411E Discovery Board

```bash
meson setup build-411 \
  --cross-file isixrtos/arm.ini \
  --cross-file isixrtos/cortex/m4.ini \
  --cross-file isixrtos/stm32/f411vet6.ini \
  -Dcrystal_hz=8000000 \
  --buildtype=release

meson compile -C build-411
```

#### STM32F469I Discovery Board

```bash
meson setup build-469 \
  --cross-file isixrtos/arm.ini \
  --cross-file isixrtos/cortex/m4.ini \
  --cross-file isixrtos/stm32/f469nih6.ini \
  -Dcrystal_hz=8000000 \
  --buildtype=release

meson compile -C build-469
```

#### BF700 Board (STM32F407ZET6)

```bash
meson setup build-bf700 \
  --cross-file isixrtos/arm.ini \
  --cross-file isixrtos/cortex/m4.ini \
  --cross-file isixrtos/stm32/f407zet6.ini \
  -Dcrystal_hz=8000000 \
  --buildtype=release

meson compile -C build-bf700
```

#### ZL41ARM Board (STM32F417VGT6)

```bash
meson setup build-zl41 \
  --cross-file isixrtos/arm.ini \
  --cross-file isixrtos/cortex/m4.ini \
  --cross-file isixrtos/stm32/f417vgt6.ini \
  -Dcrystal_hz=8000000 \
  --buildtype=release

meson compile -C build-zl41
```

### Debug Build

For debug builds, use `--buildtype=debug` or `--buildtype=custom` with optimization flags:

```bash
meson setup build-411-debug \
  --cross-file isixrtos/arm.ini \
  --cross-file isixrtos/cortex/m4.ini \
  --cross-file isixrtos/stm32/f411vet6.ini \
  -Dcrystal_hz=8000000 \
  -Db_lto=false \
  --buildtype=custom \
  --optimization=g \
  -Dcpp_rtti=false \
  -Dcpp_eh=none

meson compile -C build-411-debug
```

### Available Examples

- **stm32f411e_disco**: e0_led_blink, e1_mutex, e2_task_suspend, e3_semaphore, e4_fifo_from_irq, e5_events, e6_spi_mems, e7_i2c_mems
- **stm32f469i_disco**: e0_led_blink, e1_graphics
- **bf700**: components_demo
- **zl41arm**: blink, dacaudio

### Output Files

After building, you will find:
- ELF files: `build-*/<example_name>`
- Binary files: `build-*/<example_name>.binary`

### Notes

- The `crystal_hz` option specifies the HSE oscillator speed in Hz (default: 8000000)
- All cross files must be provided separately - Meson does not support `.ini` file inclusion
- Different build directories should be used for different boards
- The linker script and memory map are automatically configured from the cross files
