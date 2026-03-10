CXXFLAGS += -fsanitize=address -std=c++20 -Wall -ggdb -Iinclude $(shell pkg-config --cflags libftdi1)
LDFLAGS += $(shell pkg-config --libs libftdi1)

src/%.o: Makefile src/%.cpp include/mpsse_protocol.h

examples/w25qxx: src/ftdi_device.o src/mpsse_spi.o
examples/mcp9808: src/ftdi_device.o src/mpsse_i2c.o
examples/ws2812b: src/ftdi_device.o src/mpsse_ws2812b.o
examples/max31856: src/ftdi_device.o src/mpsse_spi.o

# No address sanitizer for test
ftdi_test: ftdi_test.cpp mpsse_protocol.cpp mpsse_protocol.h
	clang++ -std=c++20 -Wall -O2 $(shell pkg-config --cflags --libs libftdi1 gtest_main absl_log absl_check) ftdi_test.cpp mpsse_protocol.cpp -o $@

.PHONY: clean all
all: examples/mcp9808 examples/ws2812b examples/max31856 examples/w25qxx
clean:
	find examples -type f  ! -name "*.*" -delete
	$(RM) src/*.o
