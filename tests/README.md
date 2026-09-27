# Library tests

`test_regression.cpp` checks library functions that must behave identically in
the SIL on a PC and in the sensor firmware on the Cortex-M4F (STM32F407).

| Command | What it does |
|---|---|
| `make host` | builds and runs the tests with the host compiler (configuration like the SIL) |
| `make target OPT=-O3` | builds the tests for Cortex-M4F with the firmware's compiler flags and runs them in QEMU (`mps2-an386`) |
| `make target-compile OPT=-O0` | compiles every library source for Cortex-M4F |

Requirements for the target: `arm-none-eabi-gcc` (the CI uses Arm GNU Toolchain
13.3.rel1, see `.github/workflows/ci.yml`) and `qemu-system-arm`.

`stubs/` replaces the headers the sensor firmware provides (`system_configuration.h`,
`embedded_math.h`, ...). Note that the firmware implements `embedded_math.h` with
CMSIS-DSP functions, while the stubs use the C standard library. `target/` holds
the startup code (FPU enabled, flush-to-zero like the firmware) and the linker
script for QEMU.
