#include <arpa/inet.h>
#include <cstdio>
#include <ftdi.h>
#include <iostream>
#include <memory>
#include <chrono>
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
using mpsse_protocol::MpsseI2c;

#define MCP9808_ADDR7 0x18
#define MCP9808_REG_CONFIG 0x1
#define MCP9808_REG_TEMPERATURE 0x5
#define MCP9808_REG_MANUFACTURER_ID 0x6
#define MCP9808_REG_DEVID_REV 0x7
#define MCP9808_REG_RESOLUTION 0x8

int main(int argc, char *argv[]) {
  std::unique_ptr<FtdiDevice> dev =
      FtdiDevice::OpenVendorProduct(0x0403, 0x6010, INTERFACE_A);
  DIE_IF(dev == nullptr, "Cannot open dev");
  std::unique_ptr<MpsseI2c> i2c = MpsseI2c::Create(dev.get());
  DIE_IF(i2c == nullptr, "Cannot open i2c");

  int round = 0;
  while (1) {
    std::printf("== Round #%d ==\n", ++round);
    {
      uint8_t tx_data = MCP9808_REG_MANUFACTURER_ID;
      uint16_t rx_data;
      int success = i2c->Transaction(MCP9808_ADDR7, &tx_data, 1, &rx_data, 2);
      DIE_IF(success != 0, "Failed to get Manufacturer ID.");
      std::printf("Manufacturer ID: %#06x\n", ntohs(rx_data));
    }
    {
      uint8_t tx_data = MCP9808_REG_DEVID_REV;
      uint8_t rx_data[2];
      int success = i2c->Transaction(MCP9808_ADDR7, &tx_data, 1, &rx_data, 2);
      DIE_IF(success != 0, "Failed to get DevID / Rev.");
      std::printf("Device ID: %#04x\nRevision: %#04x\n", rx_data[0], rx_data[1]);
    }
    {
      uint8_t tx_data = MCP9808_REG_RESOLUTION;
      uint8_t rx_data;
      int success = i2c->Transaction(MCP9808_ADDR7, &tx_data, 1, &rx_data, 1);
      DIE_IF(success != 0, "Failed to get Resolution.");
      switch (rx_data) {
      case 0:  std::printf("Resolution: 0.5    °C\n"); break;
      case 1:  std::printf("Resolution: 0.25   °C\n"); break;
      case 2:  std::printf("Resolution: 0.125  °C\n"); break;
      case 3:  std::printf("Resolution: 0.0625 °C\n"); break;
      default: std::printf("Resolution: unknown(%d)\n", rx_data);
      }
    }
    {
      uint8_t tx_data = MCP9808_REG_TEMPERATURE;
      uint16_t rx_data;
      int success = i2c->Transaction(MCP9808_ADDR7, &tx_data, 1, &rx_data, 2);
      DIE_IF(success != 0, "Failed to get Temperature.");
      double temp = (double)((int16_t)(ntohs(rx_data) << 3) >> 3) / 16.0;
      printf("Temperature: (%#06x) %f°C\n", ntohs(rx_data), temp);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
