max31856: max31856.cpp mpsse_protocol.cpp mpsse_protocol.h Makefile
	clang++ -fsanitize=address -std=c++20 -Wall -ggdb $(shell pkg-config --cflags --libs libftdi1) -lm max31856.cpp mpsse_protocol.cpp -o max31856

ws2812b: ws2812b.cpp mpsse_protocol.cpp mpsse_protocol.h Makefile
	clang++ -fsanitize=address -std=c++20 -Wall -ggdb $(shell pkg-config --cflags --libs libftdi1) -lm ws2812b.cpp mpsse_protocol.cpp -o ws2812b

mcp9808: mcp9808.cpp mpsse_protocol.cpp mpsse_protocol.h Makefile
	clang++ -fsanitize=address -std=c++20 -Wall -ggdb $(shell pkg-config --cflags --libs libftdi1) -lm mcp9808.cpp mpsse_protocol.cpp -o mcp9808

.PHONY: clean all
all: mcp9808 ws2812b max31856
clean:
	$(RM) mcp9808 ws2812b max31856
