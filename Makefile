CXX_FLAGS=-fsanitize=address -std=c++20 -Wall -ggdb

mpsse_protocol.o: mpsse_protocol.cpp mpsse_protocol.h
	clang++ $(CXX_FLAGS) $(shell pkg-config --cflags libftdi1) -c $< -o $@

max31856: max31856.cpp mpsse_protocol.o
	clang++ $(CXX_FLAGS) $(shell pkg-config --cflags --libs libftdi1) $^ -o $@

ws2812b: ws2812b.cpp mpsse_protocol.o
	clang++ $(CXX_FLAGS) $(shell pkg-config --cflags --libs libftdi1) $^ -o $@

mcp9808: mcp9808.cpp mpsse_protocol.o
	clang++ $(CXX_FLAGS) $(shell pkg-config --cflags --libs libftdi1) $^ -o $@

# No address sanitizer for test
ftdi_test: ftdi_test.cpp mpsse_protocol.cpp mpsse_protocol.h
	clang++ -std=c++20 -Wall -O2 $(shell pkg-config --cflags --libs libftdi1 gtest_main absl_log absl_check) ftdi_test.cpp mpsse_protocol.cpp -o $@

.PHONY: clean all
all: mcp9808 ws2812b max31856 ftdi_test
clean:
	$(RM) mcp9808 ws2812b max31856 ftdi_test
	$(RM) *.o
