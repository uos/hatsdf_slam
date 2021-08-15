#
# Variables
#

PERCENT = %

# Tools
CXX = $(XILINX_VITIS)/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-g++
VXX = $(XILINX_VITIS)/bin/v++
HOST_CXX = g++
MKDIR_P = mkdir -p
SHELL = /bin/bash
VITIS_HLS = vitis_hls
VIVADO = vivado

# Global
BUILD_DIR = $(CURDIR)/build
APP_NAME ?= FastSense
PLATFORM_DIR ?= $(CURDIR)/base_design/platform/FastSense_platform/export/FastSense_platform

APP_EXE = $(BUILD_DIR)/$(APP_NAME).exe
APP_TEST_EXE = $(BUILD_DIR)/$(APP_NAME)_test.exe
APP_XCLBIN = $(BUILD_DIR)/$(APP_NAME).xclbin
APP_TEST_XCLBIN = $(BUILD_DIR)/$(APP_NAME)_test.xclbin

APP_IMAGE_PATH = $(CURDIR)/app_image
TEST_IMAGE_PATH = $(CURDIR)/test_image

# Add for each port from host to guest ",hostfwd=<tcp/udp>::<hostport>-192.168.1.123:<vmport>"
# and for each port from guest to host ",guestfwd=tcp:192.168.1.1:<vmport>-tcp:127.0.0.1:<hostport>"
QEMU_ARGS = -qemu-args "-netdev user,id=eth0,net=192.168.1.0/24,hostfwd=udp::2368-192.168.1.123:2368,hostfwd=tcp::1440-192.168.1.123:1534,hostfwd=tcp::5555-192.168.1.123:5555,hostfwd=tcp::6666-192.168.1.123:6666,hostfwd=tcp::7777-192.168.1.123:7777"

# Software
SYSROOT = $(PLATFORM_DIR)/sw/FastSense_platform/linux_domain/sysroot/aarch64-xilinx-linux

# Main entry point sources
ENTRY_POINT_SRCS ?= src/main.cpp
ENTRY_POINT_OBJS = $(ENTRY_POINT_SRCS:%.cpp=$(BUILD_DIR)/%.o)
ENTRY_POINT_DEPS = $(ENTRY_POINT_OBJS:.o=.d)

# Software sources
SW_SRCS = src/application.cpp \
	src/driver/imu/imu.cpp \
	src/driver/lidar/velodyne.cpp \
	$(wildcard src/map/*.cpp) \
	$(wildcard src/callback/*.cpp) \
	$(wildcard src/registration/*.cpp) \
	$(wildcard src/preprocessing/*.cpp) \
	$(wildcard src/util/*.cpp) \
	$(wildcard src/util/pcd/*.cpp) \
	$(wildcard src/eval/*.cpp) \
	$(wildcard src/msg/*.cpp) \
	$(wildcard src/util/config/*.cpp) \
	$(wildcard src/hw/*.cpp) \
	$(wildcard src/util/logging/*.cpp) \
	$(wildcard src/driver/imu/api/*.cpp) \
	$(wildcard src/ui/*.cpp) \

SW_OBJS = $(SW_SRCS:%.cpp=$(BUILD_DIR)/%.o)
SW_DEPS = $(SW_OBJS:.o=.d)

# Test sources
TEST_SRCS = $(wildcard test/*.cpp)
TEST_OBJS = $(TEST_SRCS:%.cpp=$(BUILD_DIR)/%.o)
TEST_DEPS = $(TEST_OBJS:.o=.d)

LIBS = \
	-lxilinxopencl \
	-lphidget21 \
	-lzmq \
	-lhdf5 \
	-lpthread \
	-lrt \
	-lxrt_core \
	-L$(SYSROOT)/usr/lib/ \
	-fopenmp \
	-lgpiod \
	-lgpiodcxx

INC_DIRS = \
	src \
	ext/Catch2/single_include \
	$(SYSROOT)/usr/include \
	$(SYSROOT)/usr/include/xrt \
	$(XILINX_VIVADO)/include

INC_FLAGS = $(addprefix -I,$(INC_DIRS))
CXX_STD = c++17
CXX_OPTFGLAGS ?= -O2 -ftree-loop-vectorize
GCCFLAGS = -Wall -Wextra -Wnon-virtual-dtor -ansi -pedantic -Wfatal-errors  -fexceptions -Wno-unknown-pragmas -fopenmp
CXXFLAGS = $(INC_FLAGS) $(GCCFLAGS) $(CXX_OPTFGLAGS) -MMD -MP -D__USE_XOPEN2K8 -c -fmessage-length=0 -std=$(CXX_STD) --sysroot=$(SYSROOT)

LDFLAGS = $(LIBS) --sysroot=$(SYSROOT) $(LD_EXTRA)

# Hardware
LINK_CFG = $(CURDIR)/link.cfg
LINK_TEST_CFG = $(CURDIR)/link_test.cfg
BUILD_CFG = $(CURDIR)/build.cfg
PACKAGE_CFG = $(CURDIR)/package.cfg
PACKAGE_TEST_CFG = $(CURDIR)/package_test.cfg
POST_LINK_TCL = $(CURDIR)/postSysLink.tcl

HW_TARGET ?= sw_emu
HW_PLATFORM = $(PLATFORM_DIR)/FastSense_platform.xpfm

HW_SRCS = src/registration/kernel/krnl_reg.cpp src/tsdf/krnl_tsdf.cpp
HW_OBJS = $(HW_SRCS:%.cpp=$(BUILD_DIR)/%.xo)
HW_DEPS = $(HW_OBJS:.xo=.d)

HW_TEST_SRCS = $(wildcard test/kernels/*.cpp)
HW_TEST_OBJS = $(HW_TEST_SRCS:%.cpp=$(BUILD_DIR)/%.xo)
HW_TEST_DEPS = $(HW_TEST_OBJS:.xo=.d)

HW_INC_DIRS = src
HW_INC_FLAGS = $(addprefix -I,$(HW_INC_DIRS))

ifdef PROFILING
VXX_EXTRA += --profile_kernel stall:all:all:all
VXX_LD_EXTRA += --profile_kernel data:all:all:all --profile_kernel stall:all:all:all
endif

VXXFLAGS = -t $(HW_TARGET) -f $(HW_PLATFORM) -c $(HW_INC_FLAGS) --config $(BUILD_CFG) $(VXX_EXTRA)
VXXLDFLAGS = -t $(HW_TARGET) -f $(HW_PLATFORM) --config $(LINK_CFG) --link $(VXX_LD_EXTRA)

ifeq ($(HW_TARGET), hw)
VXXLDFLAGS += --linkhook.custom postSysLink,$(POST_LINK_TCL)
endif

HW_DEPS_FLAGS = $(HW_INC_FLAGS) -isystem $(XILINX_VIVADO)/include -MM -MP

#
# Rules
#

.PHONY: all software hardware clean hls_% test clean_software clean_ros_nodes

all: software hardware

test: test_software test_hardware

clean:
	@rm -rf _x .Xil *.log *.jou pl_script.sh start_simulation.sh
	@rm -rf build/* build/.Xil
	@rm -rf app_image test_image

clean_software:
	@rm -rf $(SW_OBJS) $(ENTRY_POINT_OBJS) $(SW_DEPS) $(ENTRY_POINT_DEPS) $(APP_EXE)

clean_test:
	@rm -rf $(TEST_OBJS) $(SW_OBJS) $(TEST_DEPS) $(SW_DEPS) $(APP_TEST_EXE)

clean_hardware:
	@rm -rf $(HW_OBJS) $(HW_DEPS) $(APP_XCLBIN)

clean_ros_nodes:
	@rm -rf test/build/* test/devel/* test/*.log

software: $(APP_EXE)

hardware: $(APP_XCLBIN)

test_software: $(APP_TEST_EXE)

test_hardware: $(APP_TEST_XCLBIN)

# Link software
$(APP_EXE): $(ENTRY_POINT_OBJS) $(SW_OBJS)
	@echo "Link: $(APP_EXE)"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(ENTRY_POINT_OBJS) $(SW_OBJS) -o $@ $(LDFLAGS)

# Link test software
$(APP_TEST_EXE): $(TEST_OBJS) $(SW_OBJS)
	@echo "Link: $(APP_TEST_EXE)"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(TEST_OBJS) $(SW_OBJS) -o $@ $(LDFLAGS)

# Compile software
$(BUILD_DIR)/%.o: %.cpp
	@echo "Compile: $<"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(CXXFLAGS) $< -o $@

# Link hardware
$(APP_XCLBIN): $(HW_OBJS) $(LINK_CFG) $(POST_LINK_TCL)
	@echo "Link hardware: $(APP_XCLBIN)"
	@$(MKDIR_P) $(dir $@)
	@$(VXX) $(HW_OBJS) -o $@ $(VXXLDFLAGS) > $@.out || (cat $@.out && false)

# Link test hardware
$(APP_TEST_XCLBIN): $(HW_OBJS) $(HW_TEST_OBJS) $(LINK_CFG) $(LINK_TEST_CFG) $(POST_LINK_TCL)
	@echo "Link hardware: $(APP_TEST_XCLBIN)"
	@$(MKDIR_P) $(dir $@)
	@$(VXX) $(HW_OBJS) $(HW_TEST_OBJS) -o $@ $(VXXLDFLAGS) --config $(LINK_TEST_CFG) > $@.out || (cat $@.out && false)

# Compile kernels
$(BUILD_DIR)/%.xo: %.cpp $(BUILD_CFG)
	@echo "Compile kernel: $<"
	@$(MKDIR_P) $(dir $@)
	@$(HOST_CXX) $< $(HW_DEPS_FLAGS) -MF $(@:.xo=.d) -MT $@
	@$(VXX) $(VXXFLAGS) $< -o $@ -k $(notdir $*) > $@.out || (cat $@.out && false)

# Package SD card image
package: $(APP_XCLBIN) $(APP_EXE) $(PACKAGE_CFG) $(BUILD_DIR)/emconfig.json
	$(VXX) -p -t $(HW_TARGET) -f $(HW_PLATFORM) --config $(PACKAGE_CFG) $<

# Package SD card image for tests
package_test: $(APP_TEST_XCLBIN) $(APP_TEST_EXE) $(PACKAGE_TEST_CFG) $(BUILD_DIR)/emconfig.json
	$(VXX) -p -t $(HW_TARGET) -f $(HW_PLATFORM) --config $(PACKAGE_TEST_CFG) $<

$(BUILD_DIR)/emconfig.json:
	emconfigutil -f $(HW_PLATFORM) --od $(BUILD_DIR)

# Open HLS GUI for kernel
.SECONDEXPANSION:
hls_%: $$(filter $$(PERCENT)$$*.xo,$$(HW_OBJS) $$(HW_TEST_OBJS))
	@echo "Opening HLS for kernel $* ($^)"
	@$(VITIS_HLS) -p _x/$*/$*/$*/

open_vivado:
	@echo "Opening Vivado"
	@cd _x/link/vivado/vpl && $(VIVADO) -source openprj.tcl

copy_binaries_to_board:
	@rsync --ignore-missing-args -r $(APP_EXE) $(APP_XCLBIN) student@$(BOARD_ADDRESS):

copy_binaries_to_qemu:
	xsct -eval "set filelist {"build/FastSense.exe" "/mnt/FastSense.exe" "build/FastSense.xclbin" "/mnt/FastSense.xclbin" "app_data/config.json" "/mnt/config.json"}; source copy_to_qemu.tcl"

copy_test_to_qemu: test_software test_hardware
	xsct -eval 'set filelist {"build/FastSense_test.exe" "/mnt/FastSense_test.exe" "build/FastSense_test.xclbin" "/mnt/FastSense_test.xclbin" "test_data/config.json" "/mnt/config.json" "pcd_files/sim_cloud.pcd" "/mnt/sim_cloud.pcd" "pcd_files/robo_lab.pcd" "/mnt/robo_lab.pcd" "pcd_files/bagfile_cloud.pcd" "/mnt/bagfile_cloud.pcd"}; source copy_to_qemu.tcl'
	@echo "Successfully copied test"

start_emulator: package
	sed -i 's/ $$\*/ "$$@"/g' $(APP_IMAGE_PATH)/launch_sw_emu.sh
	$(APP_IMAGE_PATH)/launch_sw_emu.sh $(QEMU_ARGS)

start_emulator_test: package_test
	sed -i 's/ $$\*/ "$$@"/g' $(TEST_IMAGE_PATH)/launch_sw_emu.sh
	$(TEST_IMAGE_PATH)/launch_sw_emu.sh $(QEMU_ARGS)

rsync:
	@echo 'syning fastsense: to "$(USER)@$(FPGA_SERVER).informatik.uos.de:$(FGPA_SERVER_HOME)/$(USER)/fastsense"'
	@rsync -azP ./ $(USER)@$(FPGA_SERVER).informatik.uos.de:$(FGPA_SERVER_HOME)/$(USER)/fastsense

format:
	@echo "Formatting"
	@astyle -q -n --project=.astylerc --recursive "src/*.c??" "src/*.h" "test/*.c??" "test/*.h"

docs: gen_docs open_docs

gen_docs:
	@echo "Generating doxygen docs"
	@doxygen Doxygen.config

open_docs: gen_docs
	@xdg-open docs/html/index.html

-include $(ENTRY_POINT_DEPS)
-include $(SW_DEPS)
-include $(TEST_DEPS)
-include $(HW_DEPS)
-include $(HW_TEST_DEPS)
