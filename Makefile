# Host-side unit tests for the portable core/ code.
#   make test   - build and run every test
#   make clean  - delete build output
#   make sim    - build the shared library for the Python simulator

CC     = gcc
CFLAGS = -Wall -Wextra -Werror -std=c11 -Icore
LDLIBS = -lm
BUILD  = tests/build
TESTS  = $(BUILD)/test_pid $(BUILD)/test_comp_filter $(BUILD)/test_imu_convert $(BUILD)/test_roll_ctrl $(BUILD)/test_hil_proto
.PHONY: test clean sim

test: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done

$(BUILD)/test_pid: core/pid.c tests/test_pid.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/test_comp_filter: core/comp_filter.c tests/test_comp_filter.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/test_imu_convert: core/imu_convert.c tests/test_imu_convert.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/test_roll_ctrl: core/roll_ctrl.c core/pid.c tests/test_roll_ctrl.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/test_hil_proto: core/hil_proto.c tests/test_hil_proto.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD) sim/build

# Shared library for the Python simulator (same C code as the firmware)
sim/build/libcore.so: core/pid.c core/roll_ctrl.c core/hil_proto.c sim/pid_shim.c core/pid.h core/roll_ctrl.h core/hil_proto.h
	mkdir -p sim/build
	$(CC) $(CFLAGS) -shared -fPIC core/pid.c core/roll_ctrl.c core/hil_proto.c sim/pid_shim.c -o $@ $(LDLIBS)
sim: sim/build/libcore.so