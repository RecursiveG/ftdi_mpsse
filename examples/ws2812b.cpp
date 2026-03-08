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

namespace {
// HSV to RGB algo taken from https://stackoverflow.com/questions/51203917
// hsv in [0, 1), rgb in [0, 255]
void HsvToRgb(double h, double s, double v, int &r, int &g, int &b) {
    if (h < 0) h = 0;
    if (h >= 1) h = 0.9999999999;
    int i = h * 6;
    double f = h * 6 - i;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);

    switch(i) {
        case 0: r=v*255; g=t*255; b=p*255; break;
        case 1: r=q*255; g=v*255; b=p*255; break;
        case 2: r=p*255; g=v*255; b=t*255; break;
        case 3: r=p*255; g=q*255; b=v*255; break;
        case 4: r=t*255; g=p*255; b=v*255; break;
        case 5: r=v*255; g=p*255; b=q*255; break;
        default: DIE_IF(true, "impossible");
    }
}
}

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
  } else if (mode == "flow") {
    auto frame = std::make_unique<uint32_t[]>(144);
    std::memset(frame.get(), 0, 144 * 4);
    int prev = 0;
    int count = 1;
    while (true) {
      int r, g, b;
      HsvToRgb(static_cast<double>(count) / 144.0, 1, 1, r, g, b);
      frame[prev] = 0;
      frame[count] = ((r&0xff) << 16) | ((g&0xff) << 8) | (b&0xff);
      led->SendFrame({frame.get(), 144});
      prev = count;
      count = (count+1) % 144;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } else {
    std::fprintf(stderr, "Unknown mode %s\n", mode.c_str());
  }
}
