# Build and run the library regression tests.
#
#   make host            build and run on the host (SIL-like configuration)
#                        (doctest; tests in test_*.cpp, see README.md)
#   make target          build and run on a Cortex-M4F in QEMU (firmware-like configuration)
#   make target-compile  compile every library source with the firmware compiler flags
#
# OPT selects the optimization level for the target (default -O3, like the Release build).

LIB      := ..
BUILD    := build
OPT      ?= -O3

INCLUDES := -Istubs -Ithird_party/doctest -I$(LIB)/Generic_Algorithms -I$(LIB)/NAV_Algorithms \
            -I$(LIB)/Output_Formatter -I$(LIB)/Persistent_Data -I$(LIB)/ARM_math

LIB_SOURCES := $(wildcard $(LIB)/Generic_Algorithms/*.cpp $(LIB)/NAV_Algorithms/*.cpp \
                          $(LIB)/Output_Formatter/*.cpp $(LIB)/Persistent_Data/*.cpp)
ARM_MATH_SOURCES := $(wildcard $(LIB)/ARM_math/*.c)
# one test program (doctest) from all test files and the library sources they need
TEST_SOURCES := test_main.cpp test_environment.cpp test_regression.cpp test_earth_induction_model.cpp \
                $(LIB)/Output_Formatter/ascii_support.cpp $(LIB)/Persistent_Data/CRC16_table.cpp \
                $(LIB)/Persistent_Data/flexible_log_file.cpp $(LIB)/NAV_Algorithms/earth_induction_model.cpp \
                $(LIB)/NAV_Algorithms/AHRS.cpp $(LIB)/NAV_Algorithms/compass_calibrator_3D.cpp

# language standards and flags as in sw_sensor's release builds (see its README, "Release toolchain")
CXXSTD   := -std=gnu++14 -fno-exceptions -fno-rtti -DDOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS
CSTD     := -std=gnu11

# like sw_sensor's STM32CubeIDE project
TARGET_PREFIX := arm-none-eabi-
TARGET_FLAGS  := -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
# doctest on bare metal: no threads, no POSIX signals (exceptions are off via -fno-exceptions)
TARGET_DOCTEST := -DDOCTEST_CONFIG_NO_MULTITHREADING -DDOCTEST_CONFIG_NO_POSIX_SIGNALS
QEMU          := qemu-system-arm -M mps2-an386 -nographic -semihosting-config enable=on,target=native

.PHONY: host target target-compile clean

host:
	mkdir -p $(BUILD)/host_arm_math
	@for f in $(ARM_MATH_SOURCES); do \
	  $(CC) $(CSTD) -O2 $(INCLUDES) -c $$f -o $(BUILD)/host_arm_math/$$(basename $$f).o || exit 1; \
	done
	$(CXX) $(CXXSTD) -O2 -Wall -DDEVELOPMENT_ADDITIONS=1 $(INCLUDES) $(TEST_SOURCES) \
	  $(BUILD)/host_arm_math/*.o -lm -o $(BUILD)/test_host
	$(BUILD)/test_host

target:
	mkdir -p $(BUILD)/target_arm_math$(OPT)
	$(TARGET_PREFIX)gcc $(TARGET_FLAGS) -O2 $(CSTD) -c target/startup.c -o $(BUILD)/startup.o
	@for f in $(ARM_MATH_SOURCES); do \
	  $(TARGET_PREFIX)gcc $(TARGET_FLAGS) $(OPT) $(CSTD) $(INCLUDES) -c $$f -o $(BUILD)/target_arm_math$(OPT)/$$(basename $$f).o || exit 1; \
	done
	$(TARGET_PREFIX)g++ $(TARGET_FLAGS) $(OPT) $(CXXSTD) $(TARGET_DOCTEST) -Wall $(INCLUDES) --specs=rdimon.specs \
	  -T target/mps2_an386.ld -Wl,--no-warn-rwx-segments $(TEST_SOURCES) $(BUILD)/target_arm_math$(OPT)/*.o \
	  $(BUILD)/startup.o -lm -o $(BUILD)/test_target$(OPT).elf
	timeout 600 $(QEMU) -kernel $(BUILD)/test_target$(OPT).elf

target-compile:
	mkdir -p $(BUILD)/objects
	@for f in $(LIB_SOURCES); do \
	  echo "$$f"; \
	  $(TARGET_PREFIX)g++ $(TARGET_FLAGS) $(OPT) $(CXXSTD) $(INCLUDES) -c $$f -o $(BUILD)/objects/$$(basename $$f).o || exit 1; \
	done
	@for f in $(ARM_MATH_SOURCES); do \
	  echo "$$f"; \
	  $(TARGET_PREFIX)gcc $(TARGET_FLAGS) $(OPT) $(CSTD) $(INCLUDES) -c $$f -o $(BUILD)/objects/$$(basename $$f).o || exit 1; \
	done

clean:
	rm -rf $(BUILD)
