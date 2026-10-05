# Host build of the Living Map beacon protocol: tools, simulator, tests.
# Uses the SAME sources as the ESP32 firmware (../lib/BeaconProto/src).
LIB     := ../lib/BeaconProto/src
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wshadow
CPPFLAGS += -I$(LIB)
LDLIBS  := -lm
SRC_LIB := $(LIB)/beacon_proto.c $(LIB)/bp_sha256.c $(LIB)/bp_gossip.c $(LIB)/bp_json.c

FW      := ../firmware
FWSTUB  := $(FW)/host_test/stubs
CXX     ?= g++

all: bptool bpsim

bptool: bptool.c $(SRC_LIB)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^ $(LDLIBS)

bpsim: bpsim.c $(SRC_LIB)
	$(CC) $(CPPFLAGS) $(CFLAGS) -D_DEFAULT_SOURCE -o $@ $^ $(LDLIBS)

# the REAL ESP32 sketches, compiled for the PC against API stand-ins
OBJ_LIB := $(patsubst $(LIB)/%.c,obj/%.o,$(SRC_LIB))
obj/%.o: $(LIB)/%.c $(wildcard $(LIB)/*.h)
	@mkdir -p obj
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

FWFLAGS := -std=gnu++17 -O1 -Wall -Wextra -I$(FWSTUB) -I$(LIB)
fw_beacon: $(FW)/beacon_node/beacon_node.ino $(FW)/host_test/host_arduino.cpp $(OBJ_LIB) $(LIB)/bp_arduino.h
	$(CXX) $(FWFLAGS) -include Arduino.h -x c++ $< -x c++ $(FW)/host_test/host_arduino.cpp -x none $(OBJ_LIB) -o $@ -lm

fw_gateway: $(FW)/gateway_node/gateway_node.ino $(FW)/host_test/host_arduino.cpp $(OBJ_LIB) $(LIB)/bp_arduino.h
	$(CXX) $(FWFLAGS) -include Arduino.h -x c++ $< -x c++ $(FW)/host_test/host_arduino.cpp -x none $(OBJ_LIB) -o $@ -lm

firmware-host: fw_beacon fw_gateway

# Windows .exe of the tools and the simulator, cross-compiled from Linux/WSL:
#   sudo apt install gcc-mingw-w64-x86-64 && make windows
MINGW   ?= x86_64-w64-mingw32-gcc
windows: ../windows/bptool.exe ../windows/bpsim.exe
../windows/%.exe: %.c $(SRC_LIB)
	$(MINGW) -std=c99 -O2 -Wall -Wextra -I$(LIB) -s -static -o $@ $^ -lm

matrix: bpsim
	./run_matrix.sh

test: all firmware-host
	python3 ../python/tests/test_crosscheck.py
	./bpsim --quiet > /dev/null

clean:
	rm -rf bptool bpsim fw_beacon fw_gateway nvs_test.bin obj

.PHONY: all test clean firmware-host matrix windows
