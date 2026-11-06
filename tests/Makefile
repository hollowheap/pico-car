CC      ?= cc
CFLAGS  ?= -std=c99 -pedantic -Wall -Wextra -Wconversion -Wshadow -O1 -g
BUILD   := build
SRC     := ../src/physics

LOGIC   := $(SRC)/hump_detect.c $(SRC)/imu_motion.c $(SRC)/hump_msg.c
DRIVERS := $(SRC)/lsm303.c $(SRC)/motion_adapter.c $(SRC)/pipeline.c
HDRS    := $(wildcard $(SRC)/*.h)
TESTS   := $(BUILD)/test_hump_detect $(BUILD)/test_hump_msg \
           $(BUILD)/test_imu_motion $(BUILD)/test_drivers

.PHONY: all test clean

all: $(TESTS)

test: all
	./$(BUILD)/test_hump_detect
	./$(BUILD)/test_hump_msg
	./$(BUILD)/test_imu_motion
	./$(BUILD)/test_drivers

$(BUILD)/test_hump_detect: test_hump_detect.c $(SRC)/hump_detect.c $(HDRS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC) -o $@ test_hump_detect.c $(SRC)/hump_detect.c -lm

$(BUILD)/test_hump_msg: test_hump_msg.c $(SRC)/hump_msg.c $(SRC)/hump_detect.c $(HDRS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC) -o $@ test_hump_msg.c $(SRC)/hump_msg.c $(SRC)/hump_detect.c -lm

$(BUILD)/test_imu_motion: test_imu_motion.c $(SRC)/imu_motion.c $(SRC)/hump_detect.c $(HDRS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC) -o $@ test_imu_motion.c $(SRC)/imu_motion.c $(SRC)/hump_detect.c -lm

$(BUILD)/test_drivers: test_drivers.c $(LOGIC) $(DRIVERS) $(HDRS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC) -o $@ test_drivers.c $(LOGIC) $(DRIVERS) -lm

clean:
	rm -rf $(BUILD)
