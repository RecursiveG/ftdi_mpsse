#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <string>

#include <arpa/inet.h>
#include <ftdi.h>

#include "mpsse_protocol.h"

#define DIE_IF(cond, fmt, ...)                                                           \
  do {                                                                                   \
    if (cond) {                                                                          \
      fprintf(stderr, fmt "\n", ##__VA_ARGS__);                                          \
      exit(1);                                                                           \
    }                                                                                    \
  } while (0)

using mpsse_protocol::FtdiDevice;
using mpsse_protocol::MpsseWs2812b;

int main(int argc, char *argv[]) {
  std::unique_ptr<FtdiDevice> dev =
      FtdiDevice::OpenVendorProduct(0x0403, 0x6010, INTERFACE_A);
  DIE_IF(dev == nullptr, "Cannot open dev");
  std::unique_ptr<MpsseWs2812b> led = MpsseWs2812b::Create(dev.get());
  DIE_IF(led == nullptr, "Cannot open led");

  std::string mode = "blink";
  if (argc >= 2) mode = argv[1];

  if (mode == "blink") {
    uint32_t frame1[] = {0xffffff, 0};
    uint32_t frame2[] = {0, 0xffffff};
    while (1) {
      led->SendFrame(frame1);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      led->SendFrame(frame2);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } else if (mode == "1k") {
    // Norminal speed is 28.8us per LED. 1000 LED is about the limit for 30 FPS.
    auto frame = std::make_unique<uint32_t[]>(1024);
    std::memset(frame.get(), 0xff, 1024 * 4);
    for (int i = 0; i < 30; i++)
      led->SendFrame({frame.get(), 1024});
  } else {
    std::fprintf(stderr, "Unknown mode %s\n", mode.c_str());
  }
}
