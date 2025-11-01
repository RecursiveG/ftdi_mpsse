// Test & document behavior of the libftdi1 library / FT2232H

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>

#include <absl/log/check.h>
#include <absl/log/log.h>
#include <ftdi.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::ElementsAre;
typedef std::unique_ptr<struct ftdi_context, void (*)(struct ftdi_context *)>
    UniqueFtdiContext;

enum OpenMpsseMethod {
  NOT_MPSSE,
  SET_MPSSE,
};

UniqueFtdiContext OpenDevice(OpenMpsseMethod mpsse_method) {
  int err = 0;
  struct ftdi_context *ctx = ftdi_new();
  CHECK_NE(ctx, nullptr);
  err = ftdi_set_interface(ctx, INTERFACE_A);
  CHECK_EQ(err, 0);
  err = ftdi_usb_open(ctx, 0x0403, 0x6010);
  CHECK_EQ(err, 0);
  if (mpsse_method == SET_MPSSE) {
    CHECK_EQ(ftdi_set_bitmode(ctx, /*bitmask=*/0, BITMODE_MPSSE), 0);
  }
  return UniqueFtdiContext(ctx, &ftdi_free);
}

uint64_t time(std::function<void(void)> func) {
  auto start = std::chrono::high_resolution_clock::now();
  func();
  auto finish = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count();
}

int ftdi_write(struct ftdi_context *ctx, std::vector<uint8_t> data) {
  return ftdi_write_data(ctx, data.data(), data.size());
}

std::vector<uint8_t> ftdi_read(struct ftdi_context *ctx) {
  auto buf = std::make_unique<uint8_t[]>(8192);
  int r = ftdi_read_data(ctx, buf.get(), 8192);
  CHECK_GE(r, 0);

  std::vector<uint8_t> ret;
  for (int i = 0; i < r; i++) {
    ret.push_back(buf[i]);
  }
  return ret;
}

TEST(FtdiTest, StressOpenClose) {
  for (int i = 0; i < 100; i++) {
    auto ctx = OpenDevice(NOT_MPSSE);
  }
}

// Test MpsseInvalidCmdResponse
TEST(FtdiTest, MpsseInvalidCmdResponse) {
  auto buf = std::make_unique<uint8_t[]>(2048);
  auto dev = OpenDevice(SET_MPSSE);

  // Make sure we have no data to read
  ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 0);
  // Send an invalid command
  ASSERT_EQ(ftdi_write(dev.get(), {0xaa}), 1);
  // MPSSE returns us two bytes
  ASSERT_THAT(ftdi_read(dev.get()), ElementsAre(0xfa, 0xaa));
}

// Repeatedly writes 512B. Delay suddenly increases after 4K.
// FT2232 has internal buffer of 4K.
// Writes will be blocked if full.
TEST(FtdiTest, TxBufferBehavior) {
  GTEST_SKIP(); // default skip
  auto buf = std::make_unique<uint8_t[]>(512);
  auto dev = OpenDevice(NOT_MPSSE);

  for (int i = 0; i < 12; i++) {
    uint64_t t =
        time([&]() { CHECK_EQ(ftdi_write_data(dev.get(), buf.get(), 512), 512); });
    std::printf("Write %4d - %4d took %10luns\n", i * 512, i * 512 + 511, t);
  }
}

// You can split a multi-byte command into many writes.
TEST(FtdiTest, SplitCommand) {
  auto buf = std::make_unique<uint8_t[]>(2048);
  auto dev = OpenDevice(SET_MPSSE);

  // Make sure we have no data to read
  ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 0);
  // 0x22 0x00 = read one bit
  // Write first byte
  ASSERT_EQ(ftdi_write(dev.get(), {0x22}), 1);
  // Shouldn't read any data yet.
  LOG(INFO) << "Waiting...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 0);
  // Write second byte
  ASSERT_EQ(ftdi_write(dev.get(), {0x00}), 1);
  // Will get data back.
  ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 1);
}

// According to the datasheet, the HW structure is like
// USB --> 4K TX Buffer --> MPSSE
//    <--  4K RX Buffer <--
// Looks like the partial command is latched inside the MPSSE.
// Reopen the USB device only clears the two buffers and MPSSE is not reset.
// Power cycle (unplug then replug the USB) or reset the bitbang mode can fix that.
TEST(FtdiTest, SplitCommandPersistsAcrossReset) {
  auto buf = std::make_unique<uint8_t[]>(2048);
  {
    auto dev = OpenDevice(SET_MPSSE);

    // Make sure we have no data to read
    ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 0);
    // 0x20 0xaa 0x00 = read 171 bytes
    // Write first byte
    ASSERT_EQ(ftdi_write(dev.get(), {0x20}), 1);
    // Shouldn't read any data yet.
    LOG(INFO) << "Waiting...";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 0);
  }
  // Close and reopen device
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  {
    auto dev = OpenDevice(SET_MPSSE);

    // still no data
    ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 0);
    // Write second and third byte
    ASSERT_EQ(ftdi_write(dev.get(), {0xaa, 0x00}), 2);
    // Now we get data back.
    ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 171);
  }
}

// Test bit mode reset
TEST(FtdiTest, SplitCommandNotPersistsAcrossResetBitbangMode) {
  auto buf = std::make_unique<uint8_t[]>(2048);
  auto dev = OpenDevice(SET_MPSSE);

  // Make sure we have no data to read
  ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 0);
  // 0x20 0xaa 0x00 = read 171 bytes
  // Write first byte
  ASSERT_EQ(ftdi_write(dev.get(), {0x20}), 1);
  // Shouldn't read any data yet.
  LOG(INFO) << "Waiting...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 0);

  // Reset bitbang
  ASSERT_EQ(ftdi_set_bitmode(dev.get(), 0, BITMODE_RESET), 0);
  ASSERT_EQ(ftdi_set_bitmode(dev.get(), 0, BITMODE_MPSSE), 0);

  // Write second and third byte
  ASSERT_EQ(ftdi_write(dev.get(), {0xaa, 0x00}), 2);
  // Now we get data back, but the two bytes are treated as two unknown commands
  ASSERT_THAT(ftdi_read(dev.get()), ElementsAre(0xfa, 0xaa, 0xfa, 0x00));
}
