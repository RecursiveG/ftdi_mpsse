// Test program for SPI Flash.
// Tested on Winbond W25Q64FV.
// Use "flashrom" for real stuff.
// This program will erase data on the flash.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include "mpsse_protocol.h"

#define DIE_IF(cond, fmt, ...)                                                                          \
  do {                                                                                                  \
    if (cond) {                                                                                         \
      fprintf(stderr, fmt "\n", ##__VA_ARGS__);                                                         \
      exit(1);                                                                                          \
    }                                                                                                   \
  } while (0)

using mpsse_protocol::FtdiDevice;
using mpsse_protocol::MpsseSpi;
using mpsse_protocol::Status;

Status CmdJedecId(MpsseSpi *spi, uint8_t &manuf_id, uint8_t &mem_type, uint32_t &size_bytes) {
  uint8_t cmd[] = {0x9f};
  uint32_t out;
  Status st = spi->Transaction(cmd, 1, &out, 3);
  if (!st.ok()) return st;
  manuf_id = out;
  mem_type = out >> 8;
  size_bytes = 1 << ((out >> 16) & 0xff);
  return Status::Ok();
}

void hexdump(const void *buf, int len) {
  for (int i = 0; i < len; i++) {
    if (i > 0 && i % 32 == 0) {
      std::printf("\n");
    }
    if (i % 32 != 0 && i % 16 == 0) {
      std::printf(" ");
    }
    std::printf("%02x", (static_cast<const uint8_t *>(buf))[i]);
  }
  std::printf("\n");
}

void EnableWriteOnce(MpsseSpi *spi) {
  uint8_t cmd = 0x6;
  Status st = spi->Transaction(&cmd, 1, nullptr, 0);
  DIE_IF(!st.ok(), "Failed to enable write %s", st.human().c_str());
}

bool Busy(MpsseSpi *spi) {
  uint8_t cmd = 0x5;
  uint8_t sta = 0;
  Status st = spi->Transaction(&cmd, 1, &sta, 1);
  DIE_IF(!st.ok(), "Failed to read status-1 %s", st.human().c_str());
  // std::printf("Status1: %#x\n", sta);
  return (sta & 1);
}

int main(int argc, char *argv[]) {
  std::unique_ptr<FtdiDevice> dev = FtdiDevice::OpenVendorProduct(0x0403, 0x6010, INTERFACE_A);
  DIE_IF(dev == nullptr, "Cannot open dev");
  // W25Q64FV supports 0,0 or 1,1
  // 30 MHz is the max speed that FT2232 can go.
  std::unique_ptr<MpsseSpi> spi = MpsseSpi::Create(dev.get(), 0, 0, 30);
  DIE_IF(spi == nullptr, "Cannot open SPI");

  // Read JEDEC ID
  uint8_t manuf_id;
  uint8_t memory_type;
  uint32_t size_bytes;
  Status st = CmdJedecId(spi.get(), manuf_id, memory_type, size_bytes);
  DIE_IF(!st.ok(), "Read ID failed: %s", st.human().c_str());
  std::printf("Manuf: %#x, Type: %#x, Size: %.01f MiB\n", manuf_id, memory_type,
              static_cast<double>(size_bytes) / 1024 / 1024);
  DIE_IF(manuf_id != 0xef || memory_type != 0x40, "Unexpected Flash module.");

  if (argc < 2) {
    std::printf("need op\n");
    return 0;
  }
  std::string op = argv[1];

  if (op == "read1k") {
    // Read 1KB
    // No need to use the fast read command, it's only needed if SPI is beyond 50 MHz.
    uint8_t cmd[] = {0x3, 0, 0, 0};
    auto buffer = std::make_unique<uint8_t[]>(1024);
    st = spi->Transaction(cmd, 4, buffer.get(), 1024);
    DIE_IF(!st.ok(), "Read data failed: %s", st.human().c_str());
    hexdump(buffer.get(), 1024);
  }

  if (op == "read_bench") {
    // Read all pages
    auto buffer = std::make_unique<uint8_t[]>(65536);
    auto start = std::chrono::high_resolution_clock::now();
    for (unsigned int i = 0; i < size_bytes / 65536; i++) {
      std::printf("Reading block %d...\n", i);
      uint8_t cmd[4];
      cmd[0] = 0x3;
      cmd[1] = (i * 65536) >> 16;
      cmd[2] = (i * 65536) >> 8;
      cmd[3] = (i * 65536);
      st = spi->Transaction(cmd, 4, buffer.get(), 65536);
      DIE_IF(!st.ok(), "Read data failed: %s", st.human().c_str());
    }
    double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start)
            .count();
    std::printf("Reading %d bytes, took %.2lf ms, speed %.2fKiB/s\n", size_bytes, elapsed_ms,
                static_cast<double>(size_bytes) / elapsed_ms * 1000 / 1024);
  }

  if (op == "write_page") {
    // one page is 256 bytes.
    auto buffer = std::make_unique<uint8_t[]>(4 + 256);
    buffer[0] = 0x2;
    buffer[1] = 0;
    buffer[2] = 1;
    buffer[3] = 0;
    for (int i = 0; i < 256; i++)
      buffer[4 + i] = 0xfd;
    EnableWriteOnce(spi.get());
    st = spi->Transaction(buffer.get(), 4 + 256, nullptr, 0);
    DIE_IF(!st.ok(), "page write failed: %s", st.human().c_str());
    std::printf("page write cmd issued\n");
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (!Busy(spi.get())) break;
    }
    std::printf("page write complete\n");
  }

  if (op == "verify_block") {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> random_byte{};
    constexpr uint32_t kBlockAddr = 0; // block address in bytes.

    auto buffer = std::make_unique<uint8_t[]>(65536);
    for (int i = 0; i < 65536; i++)
      buffer[i] = random_byte(gen);

    // Erase block
    EnableWriteOnce(spi.get());
    uint8_t cmd[] = {0xd8, kBlockAddr >> 16, kBlockAddr >> 8, kBlockAddr};
    st = spi->Transaction(cmd, 4, nullptr, 0);
    DIE_IF(!st.ok(), "Erase block cmd failed: %s", st.human().c_str());
    std::printf("Erasing block %#x\n", kBlockAddr);
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (!Busy(spi.get())) break;
    }
    std::printf("Block erased\n");

    // Write data page by page
    for (int i = 0; i < 65536 / 256; i++) {
      std::printf("Writing page %d...\n", i);
      uint32_t addr = kBlockAddr + i * 256;
      auto write_cmd = std::make_unique<uint8_t[]>(4 + 256);
      write_cmd[0] = 0x2;
      write_cmd[1] = addr >> 16;
      write_cmd[2] = addr >> 8;
      write_cmd[3] = addr;
      for (int j = 0; j < 256; j++)
        write_cmd[4 + j] = buffer[i * 256 + j];

      EnableWriteOnce(spi.get());
      st = spi->Transaction(write_cmd.get(), 4 + 256, nullptr, 0);
      DIE_IF(!st.ok(), "Write page cmd failed: %s", st.human().c_str());
      while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!Busy(spi.get())) break;
      }
    }

    // Read data back
    auto read_buf = std::make_unique<uint8_t[]>(65536);
    std::printf("Reading block...\n");
    cmd[0] = 0x3;
    cmd[1] = kBlockAddr >> 16;
    cmd[2] = kBlockAddr >> 8;
    cmd[3] = kBlockAddr;
    st = spi->Transaction(cmd, 4, read_buf.get(), 65536);
    DIE_IF(!st.ok(), "Read block failed: %s", st.human().c_str());

    // Compare
    int err_bytes = 0;
    int err_bits = 0;
    for (int i = 0; i < 65536; i++) {
      if (read_buf[i] == buffer[i]) continue;
      err_bytes += 1;
      err_bits += __builtin_popcount(read_buf[i] ^ buffer[i]);
    }
    std::printf("Verify byte diff %d, bit diff %d\n", err_bytes, err_bits);
    std::printf("First two pages:\n");
    hexdump(read_buf.get(), 512);
  }

  if (op == "verify_all") {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> random_byte{};
    auto buffer = std::make_unique<uint8_t[]>(size_bytes);
    for (unsigned int i = 0; i < size_bytes; i++)
      buffer[i] = random_byte(gen);

    // Chip erase
    auto start = std::chrono::high_resolution_clock::now();
    EnableWriteOnce(spi.get());
    // 3 dummy bytes to make an array of size 4, which is needed later.
    uint8_t cmd[4] = {0x60, 0, 0, 0};
    st = spi->Transaction(cmd, 1, nullptr, 0);
    DIE_IF(!st.ok(), "Erase chip cmd failed: %s", st.human().c_str());
    std::printf("Erasing chip...\n");
    while (true) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (!Busy(spi.get())) break;
    }
    std::printf(
        "Block erased, took %.2lf\n",
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count());

    // Write data page by page
    start = std::chrono::high_resolution_clock::now();
    std::printf("Writing pages\n");
    for (unsigned int i = 0; i < size_bytes / 256; i++) {
      std::printf("\33[2K\rPage %5d/%d", i, size_bytes / 256);
      std::fflush(stdout);
      uint32_t addr = i * 256;
      auto write_cmd = std::make_unique<uint8_t[]>(4 + 256);
      write_cmd[0] = 0x2;
      write_cmd[1] = addr >> 16;
      write_cmd[2] = addr >> 8;
      write_cmd[3] = addr;
      for (int j = 0; j < 256; j++)
        write_cmd[4 + j] = buffer[i * 256 + j];

      EnableWriteOnce(spi.get());
      st = spi->Transaction(write_cmd.get(), 4 + 256, nullptr, 0);
      DIE_IF(!st.ok(), "Write page cmd failed: %s", st.human().c_str());
      while (true) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        if (!Busy(spi.get())) break;
      }
    }
    std::printf("\ndone!\n");
    double elapsed_s =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    std::printf("Written %d bytes in %.2lf sec, speed %.2lf KiB/s\n", size_bytes, elapsed_s,
                static_cast<double>(size_bytes) / elapsed_s / 1024);

    // Read data back and compare.
    int err_bytes = 0;
    int err_bits = 0;
    auto read_buf = std::make_unique<uint8_t[]>(65536);
    start = std::chrono::high_resolution_clock::now();
    std::printf("Reading blocks");
    std::fflush(stdout);
    for (unsigned int i = 0; i < size_bytes / 65536; i++) {
      std::printf(".");
      std::fflush(stdout);
      cmd[0] = 0x3;
      cmd[1] = (i * 65536) >> 16;
      cmd[2] = (i * 65536) >> 8;
      cmd[3] = (i * 65536);
      st = spi->Transaction(cmd, 4, read_buf.get(), 65536);
      DIE_IF(!st.ok(), "Read block failed: %s", st.human().c_str());

      // Compare
      for (int j = 0; j < 65536; j++) {
        if (read_buf[j] == buffer[i * 65536 + j]) continue;
        err_bytes += 1;
        err_bits += __builtin_popcount(read_buf[j] ^ buffer[i * 65536 + j]);
      }
    }
    elapsed_s = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    std::printf("done!\nVerify byte diff %d, bit diff %d\n", err_bytes, err_bits);
    std::printf("Verified %d bytes in %.2lf sec, speed %.2lf MiB/s\n", size_bytes, elapsed_s,
                static_cast<double>(size_bytes) / elapsed_s / 1024 / 1024);
  }

  return 0;
}
