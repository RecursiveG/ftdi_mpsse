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
using ::testing::IsEmpty;
using ::testing::SizeIs;

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

// Clear all buffers, reset MPSSE, then set clock to 1kHz
void ftdi_init_clk1k(struct ftdi_context *ctx) {
  CHECK_EQ(ftdi_tcoflush(ctx), 0);
  CHECK_EQ(ftdi_set_bitmode(ctx, 0, BITMODE_RESET), 0);
  CHECK_EQ(ftdi_set_bitmode(ctx, 0, BITMODE_MPSSE), 0);
  CHECK_EQ(ftdi_tciflush(ctx), 0);
  // Set to 1000 Hz
  CHECK_EQ(ftdi_write(ctx, {0x8d, 0x97, 0x8a, 0x86, 0x2f, 0x75}), 6);
  CHECK_EQ(ftdi_read(ctx).size(), 0);
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

// Test MpsseInvalidCmdResponse, but the response is discarded using tciflush
TEST(FtdiTest, MpsseInvalidCmdResponseButDiscardedWithTciflush) {
  auto dev = OpenDevice(SET_MPSSE);

  // Make sure we have no data to read
  ASSERT_THAT(ftdi_read(dev.get()), SizeIs(0));
  // Send an invalid command
  ASSERT_EQ(ftdi_write(dev.get(), {0xaa}), 1);
  // FIXME: what happens if the flush happens before the data is written back?
  ASSERT_EQ(ftdi_tciflush(dev.get()), 0);
  // MPSSE returns us zero bytes
  ASSERT_THAT(ftdi_read(dev.get()), SizeIs(0));
}

// Repeatedly writes 512B. Delay suddenly increases after 4K.
// FT2232 has internal buffer of 4K.
// Writes will be blocked if full.
// default disabled because it desync MPSSE
TEST(FtdiTest, DISABLED_TxWriteTimeJumpAt4K) {
  auto buf = std::make_unique<uint8_t[]>(512);
  auto dev = OpenDevice(NOT_MPSSE);

  for (int i = 0; i < 12; i++) {
    uint64_t t =
        time([&]() { CHECK_EQ(ftdi_write_data(dev.get(), buf.get(), 512), 512); });
    std::printf("Write %4d - %4d took %10luns\n", i * 512, i * 512 + 511, t);
  }
}

// Not really some useful checks. Just see what's the value.
// I think this is the usb transfer size.
TEST(FtdiTest, ChunkSizeRead) {
  uint32_t chunksize = 0;
  auto dev = OpenDevice(NOT_MPSSE);
  ASSERT_EQ(ftdi_read_data_get_chunksize(dev.get(), &chunksize), 0);
  EXPECT_EQ(chunksize, 4096);
}
TEST(FtdiTest, ChunkSizeWrite) {
  uint32_t chunksize = 0;
  auto dev = OpenDevice(NOT_MPSSE);
  ASSERT_EQ(ftdi_write_data_get_chunksize(dev.get(), &chunksize), 0);
  EXPECT_EQ(chunksize, 4096);
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

// check the modem_status u16 at 4 points
// - just started, all idle
// - a command in progress
// - command finished, but data not read
// - after data read.
// It looks like all status returned is 0x6032. I don't know why.
TEST(FtdiTest, SplitCommandModemStatus) {
  auto dev = OpenDevice(SET_MPSSE);
  const uint16_t expected_status = 0x6032;
  uint16_t status = 0;

  // Make sure we have no data to read
  ASSERT_THAT(ftdi_read(dev.get()), SizeIs(0));
  // Check status when idle
  ASSERT_EQ(ftdi_poll_modem_status(dev.get(), &status), 0);
  EXPECT_EQ(status, expected_status);
  // std::printf("Idle status %#06x\n", status);

  // 0x22 0x00 = read one bit
  // Write first byte
  ASSERT_EQ(ftdi_write(dev.get(), {0x22}), 1);
  // Shouldn't have any data yet.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_THAT(ftdi_read(dev.get()), SizeIs(0));
  // check status
  status = 0;
  ASSERT_EQ(ftdi_poll_modem_status(dev.get(), &status), 0);
  EXPECT_EQ(status, expected_status);
  // std::printf("Blocked status %#06x\n", status);

  // Write second byte
  ASSERT_EQ(ftdi_write(dev.get(), {0x00}), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Check status
  status = 0;
  ASSERT_EQ(ftdi_poll_modem_status(dev.get(), &status), 0);
  EXPECT_EQ(status, expected_status);
  // std::printf("Before read status %#06x\n", status);

  // Now get data back and check status
  ASSERT_THAT(ftdi_read(dev.get()), SizeIs(1));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  status = 0;
  ASSERT_EQ(ftdi_poll_modem_status(dev.get(), &status), 0);
  EXPECT_EQ(status, expected_status);
  // std::printf("After read status %#06x\n", status);
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

// Test tx purge has no effect on partial commands
TEST(FtdiTest, SplitCommandNotAffectedByTxPurge) {
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
  // TX Purge
  ASSERT_EQ(ftdi_tcoflush(dev.get()), 0);
  // Write second and third byte
  ASSERT_EQ(ftdi_write(dev.get(), {0xaa, 0x00}), 2);
  // Now we get data back.
  ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 512), 171);
}

// Test read duration at a given frequency
// Result approx: 1.5199s
TEST(FtdiTest, ReadTiming) {
  auto dev = OpenDevice(NOT_MPSSE);
  ftdi_init_clk1k(dev.get());

  // Read 187 bytes, should take ~1.5s, try 10 times
  for (int i = 0; i < 10; i++) {
    ASSERT_EQ(ftdi_write(dev.get(), {0x20, 0xba, 0x00}), 3);
    std::vector<uint8_t> r;
    double t = time([&]() { r = ftdi_read(dev.get()); });
    ASSERT_EQ(r.size(), 187);
    LOG(INFO) << "187 bytes took sec=" << t / 1e9;
  }
}

// Check if "Send Immediate 0x87" has any effect on reading.
// Result: approx: 1.5119s.
//         Yes noticable difference.
// But is it mpsse flush to rxbuffer or rxbuffer flush to usb?
TEST(FtdiTest, ReadTimingWithSendImmediate) {
  auto dev = OpenDevice(NOT_MPSSE);
  ftdi_init_clk1k(dev.get());

  // Read 187 bytes, should take ~1.5s, try 10 times
  for (int i = 0; i < 10; i++) {
    ASSERT_EQ(ftdi_write(dev.get(), {0x20, 0xba, 0x00, 0x87}), 4);
    std::vector<uint8_t> r;
    double t = time([&]() { r = ftdi_read(dev.get()); });
    ASSERT_EQ(r.size(), 187);
    LOG(INFO) << "187 bytes took sec=" << t / 1e9;
  }
}

// Only read 4bytes a time
// Result: except the first read took 47.9 ms, all other took ~32ms
//         expected time is 32ms.
//         unsure if the first read is due to cold code path.
//         Adding a warmup call doesn't seem to fix it.
TEST(FtdiTest, ReadTimingWith4BReads) {
  auto buf = std::make_unique<uint8_t[]>(4);
  auto dev = OpenDevice(NOT_MPSSE);
  ftdi_init_clk1k(dev.get());

  // ASSERT_EQ(ftdi_read_data(dev.get(), buf.get(), 4), 0);
  // Attempt to warmup, but no effect?

  // Read 128 bytes as 32x 4byte-reads
  ASSERT_EQ(ftdi_write(dev.get(), {0x20, 0x7f, 0x00}), 3);
  for (int i = 0; i < 32; i++) {
    int r = 0;
    double t = time([&]() { r = ftdi_read_data(dev.get(), buf.get(), 4); });
    ASSERT_EQ(r, 4);
    LOG(INFO) << "4 bytes took milisec=" << t / 1e6;
  }
}

// 4k reads, with send immediate
// Result: 1st 4k 47.9ms, middle 31.9ms, last 16ms.
//         it looks like MPSSE actully buffers some bytes internally and
//         send immediate flushes them to the rxbuf
TEST(FtdiTest, ReadTimingWith4BReadsWithSendImmediate) {
  auto buf = std::make_unique<uint8_t[]>(4);
  auto dev = OpenDevice(NOT_MPSSE);
  ftdi_init_clk1k(dev.get());

  // Read 128 bytes as 32x 4byte-reads
  ASSERT_EQ(ftdi_write(dev.get(), {0x20, 0x7f, 0x00, 0x87}), 4);
  for (int i = 0; i < 32; i++) {
    int r = 0;
    double t = time([&]() { r = ftdi_read_data(dev.get(), buf.get(), 4); });
    ASSERT_EQ(r, 4);
    LOG(INFO) << "4 bytes took milisec=" << t / 1e6;
  }
}

void TimeBlockedRead(int total, int block, int latency, bool imm) {
  auto buf = std::make_unique<uint8_t[]>(block);
  auto dev = OpenDevice(NOT_MPSSE);
  ftdi_init_clk1k(dev.get());
  ASSERT_EQ(ftdi_set_latency_timer(dev.get(), latency), 0);
  LOG(INFO) << "Latency set to " << latency << "ms";

  if (imm) {
    ASSERT_EQ(
        ftdi_write(dev.get(), {0x20, static_cast<uint8_t>((total - 1) & 0xff),
                               static_cast<uint8_t>(((total - 1) >> 8) & 0xff), 0x87}),
        4);
  } else {
    ASSERT_EQ(ftdi_write(dev.get(), {0x20, static_cast<uint8_t>((total - 1) & 0xff),
                                     static_cast<uint8_t>(((total - 1) >> 8) & 0xff)}),
              3);
  }

  int r_total = 0;
  int skipped_read = 0;
  auto t_begin = std::chrono::high_resolution_clock::now();
  while (r_total < total) {
    int r = ftdi_read_data(dev.get(), buf.get(), block);
    auto t_end = std::chrono::high_resolution_clock::now();
    if (r == 0) {
      skipped_read++;
      continue;
    }
    ASSERT_GT(r, 0);
    double elap =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_begin).count();
    LOG(INFO) << "Read " << r << "bytes skipped " << skipped_read
              << " took ms=" << elap / 1e6;
    r_total += r;
    t_begin = t_end;
    skipped_read = 0;
  }
}

// Test various combination of block size and latency timer.
// Result: Changing latency timer do have effect on MPSSE

// Read call returns every byte (even 4 requested)
// Also many 0 bytes reads in between
TEST(FtdiTest, TimeBlockedRead4B2MSImm) { TimeBlockedRead(128, 4, 2, true); }
// Read call returns every byte (even 4 requested)
// Reduced 0 bytes reads in between
TEST(FtdiTest, TimeBlockedRead4B4MSImm) { TimeBlockedRead(128, 4, 4, true); }
// First, mid, last = 39.8, 32, 24.2
// Have one 0B read at the beginning, but all other returns 4B together.
TEST(FtdiTest, TimeBlockedRead4B8MSImm) { TimeBlockedRead(128, 4, 8, true); }
// First, mid, last = 48, 32, 16
TEST(FtdiTest, TimeBlockedRead4B16MSImm) { TimeBlockedRead(128, 4, 16, true); }
// First, mid, last = 64, 32, 0
TEST(FtdiTest, TimeBlockedRead4B32MSImm) { TimeBlockedRead(128, 4, 32, true); }
// Timing become choppy: 48,48,48,0,48,48,0,48,48,...
// I think this is libftdi1 doing buffer internally.
// every 48ms gives 6bytes, so two 48ms wait gives 12B and the 3rd 4B read can return
// immediately.
TEST(FtdiTest, TimeBlockedRead4B48MSImm) { TimeBlockedRead(128, 4, 48, true); }
// Very choppy, 64,0,64,0,...
// Last 4B read is immediate due to the IMM command.
TEST(FtdiTest, TimeBlockedRead4B64MSImm) { TimeBlockedRead(128, 4, 64, true); }
// Same as above, but last 4B read need to wait 64ms before return.
TEST(FtdiTest, TimeBlockedRead4B64MS) { TimeBlockedRead(128, 4, 64, false); }
