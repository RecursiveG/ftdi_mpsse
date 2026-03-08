#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <ftdi.h>
#include <iostream>
#include <memory>
#include <thread>

#include "mpsse_protocol.h"

#define DIE_IF(cond, fmt, ...)                                                           \
  do {                                                                                   \
    if (cond) {                                                                          \
      fprintf(stderr, fmt "\n", ##__VA_ARGS__);                                          \
      exit(1);                                                                           \
    }                                                                                    \
  } while (0)

using mpsse_protocol::FtdiDevice;
using mpsse_protocol::MpsseSpi;

#define MAX31856_WRITE 0x80
#define MAX31856_CONFIG0 0x00
#define MAX31856_CONFIG1 0x01

int main(int argc, char *argv[]) {
  std::unique_ptr<FtdiDevice> dev =
      FtdiDevice::OpenVendorProduct(0x0403, 0x6010, INTERFACE_A);
  DIE_IF(dev == nullptr, "Cannot open dev");
  std::unique_ptr<MpsseSpi> spi = MpsseSpi::Create(dev.get(), 1, 1);
  DIE_IF(spi == nullptr, "Cannot open SPI");

  int err;
  uint8_t data[6];
  // Write CR1
  data[0] = MAX31856_CONFIG1 | MAX31856_WRITE;
  data[1] = 0x23; // TypeK, 4sample avg, needs 243ms
  err = spi->Transaction(data, 2, nullptr, 0);
  DIE_IF(err != 0, "SPI Transaction failed: CR1");

  while(true) {
    // Trigger
    data[0] = MAX31856_CONFIG0 | MAX31856_WRITE;
    data[1] = 0x40; // One shot
    err = spi->Transaction(data, 2, nullptr, 0);
    DIE_IF(err != 0, "SPI Transaction failed: CR0");

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Read
    data[0] = 0xa;
    err = spi->Transaction(data, 1, data + 1, 5);
    DIE_IF(err != 0, "SPI Transaction failed: Read");

    //
    int16_t cj_data = (data[1] << 8) | data[2];
    int32_t tc_data = (data[3] << 24) | (data[4] << 16) | (data[5] << 8);
    // std::printf("Raw value %#x %#x\n", cj_data, tc_data);
    std::printf("CJ-TC: %9.2f %9.2f °C\n", static_cast<float>(cj_data >> 2) / 64,
                static_cast<float>(tc_data >> 13) / 128);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}
