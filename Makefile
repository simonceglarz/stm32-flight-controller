# Host-side unit tests for the portable core/ code.
#   make test   - build and run every test
#   make clean  - delete build output

CC     = gcc
CFLAGS = -Wall -Wextra -Werror -std=c11 -Icore   # -Werror: any warning fails the build
LDLIBS = -lm                                     # math library (fabsf, atan2f)
BUILD  = tests/build
TESTS  = $(BUILD)/test_pid $(BUILD)/test_comp_filter $(BUILD)/test_imu_convert

.PHONY: test clean

test: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done

$(BUILD)/test_pid: core/pid.c tests/test_pid.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/test_comp_filter: core/comp_filter.c tests/test_comp_filter.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/test_imu_convert: core/imu_convert.c tests/test_imu_convert.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)

# Shared library for the Python simulator (same C code as the firmware)
sim/build/libcore.so: core/pid.c sim/pid_shim.c core/pid.h
	mkdir -p sim/build
	$(CC) $(CFLAGS) -shared -fPIC core/pid.c sim/pid_shim.c -o $@ $(LDLIBS)

.PHONY: sim
sim: sim/build/libcore.so