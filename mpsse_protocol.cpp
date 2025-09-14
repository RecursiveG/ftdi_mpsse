#include "mpsse_protocol.h"

#include <cmath>
#include <cstdio>
#include <chrono>
#include <memory>
#include <functional>
#include <ftdi.h>

#define RETURN_IF(cond, ret, fmt, ...)                                                   \
  do {                                                                                   \
    if (cond) {                                                                          \
      std::fprintf(stderr, fmt "\n", ##__VA_ARGS__);                                     \
      return (ret);                                                                      \
    }                                                                                    \
  } while (0)

namespace mpsse_protocol {

std::unique_ptr<FtdiDevice> FtdiDevice::OpenVendorProduct(uint16_t id_vendor,
                                                          uint16_t id_product,
                                                          enum ftdi_interface intf) {
  int err = 0;
  struct ftdi_context *ctx = nullptr;

  ctx = ftdi_new();
  RETURN_IF(ctx == NULL, nullptr, "ftdi_new() failed");
  err = ftdi_set_interface(ctx, intf);
  if (err) { ftdi_free(ctx); std::fprintf(stderr, "ftdi_set_interface() failed: %d\n", err); return nullptr;}
  err = ftdi_usb_open(ctx, id_vendor, id_product);
  if (err) { ftdi_free(ctx); std::fprintf(stderr, "ftdi_usb_open() failed: %d\n", err); return nullptr;}

  return std::make_unique<FtdiDevice>(ctx);
}

std::unique_ptr<FtdiDevice> FtdiDevice::OpenBusDevice(int bus, int device,
                                                      enum ftdi_interface intf) {
  int err = 0;
  struct ftdi_context *ctx = nullptr;

  ctx = ftdi_new();
  RETURN_IF(ctx == NULL, nullptr, "ftdi_new() failed");
  err = ftdi_set_interface(ctx, intf);
  if (err) { ftdi_free(ctx); std::fprintf(stderr, "ftdi_set_interface() failed: %d\n", err); return nullptr;}
  err = ftdi_usb_open_bus_addr(ctx, bus, device);
  if (err) { ftdi_free(ctx); std::fprintf(stderr, "ftdi_usb_open_bus_addr() failed: %d\n", err); return nullptr;}

  return std::make_unique<FtdiDevice>(ctx);
}

FtdiDevice::~FtdiDevice() {
  if (context_) {
    ftdi_free(context_);
  }
}

void FtdiDevice::BufferClear() { buffer_len_ = 0; }
int FtdiDevice::BufferByte(uint8_t data) {
  RETURN_IF(buffer_len_ >= kBufferSize, -1, "Too many data in buffer");
  buffer_[buffer_len_++] = data;
  return 0;
}
int FtdiDevice::BufferBytes(std::span<uint8_t> data) {
  RETURN_IF(buffer_len_ + data.size() > kBufferSize, -1, "Too many bytes to write");
  for (ssize_t i = 0; i < data.size(); i++) {
    buffer_[buffer_len_++] = data[i];
  }
  return 0;
}
int FtdiDevice::BufferFlush() {
  if (buffer_len_ == 0)
    return 0;
  int ret = ftdi_write_data(context_, buffer_, buffer_len_);
  RETURN_IF(ret != buffer_len_, -1, "ftdi_write_data() failed: expect %d got %d",
            buffer_len_, ret);
  buffer_len_ = 0;
  return 0;
}

int FtdiDevice::Read(void *buf, int32_t len) {
    auto begin = std::chrono::high_resolution_clock::now();
    auto deadline = begin + std::chrono::milliseconds{1};
    auto* ptr = static_cast<unsigned char*>(buf);
    do {
        int read = ftdi_read_data(context_, static_cast<unsigned char*>(ptr), len);
        if (read == 0) continue;
        RETURN_IF(read < 0, -1, "ftdi_read_bytes() failed: %d", read);
        len -= read;
        RETURN_IF(len < 0, -1, "ftdi_read_bytes() got too many bytes %d", len);
        if (len == 0) return 0;
        ptr += read;
    } while(std::chrono::high_resolution_clock::now() < deadline);
    RETURN_IF(true, -1, "ftdi_read_bytes() timed out");
}

int FtdiDevice::MpsseSync() {
  constexpr int kBufSize = 256;
  uint8_t out_data[] = {0xab, 0xaa}; // two bad commands
  uint8_t buf[kBufSize];
  uint32_t in_data = 0;
  int ret;

  // Write two commands, expect echo back.
  ret = ftdi_write_data(context_, out_data, 2);
  RETURN_IF(ret != 2, -1, "ftdi_write_data() failed: %d", ret);

  // Spin for at least 100us, if data not ready, spin up to 10ms.
  auto begin = std::chrono::high_resolution_clock::now();
  auto GetElapsedUs = [&begin]() -> uint64_t {
    return std::chrono::duration<double, std::micro>(
               std::chrono::high_resolution_clock::now() - begin).count();
  };
  while (true) {
    uint64_t elapsed = GetElapsedUs();
    if (elapsed > 10'000) break;
    if (elapsed >    100 && in_data == 0xfaabfaaa) break;

    ret = ftdi_read_data(context_, buf, kBufSize);
    RETURN_IF(ret < 0, -1, "ftdi_read_data() failed: %d", ret);
    if (ret > 0) {
      for (int i = 0; i < std::fmin(ret, 4); i++) {
        // Sliding window looking for the last 4 bytes returned.
        in_data = (in_data << 8) | buf[i];
      }
    }
  }

  RETURN_IF(in_data != 0xfaabfaaa, -1, "MPSSE synchronization failed");
  return 0;
}

int FtdiDevice::MpsseSetClockFreq(float khz, bool three_phase, bool adaptive) {
  RETURN_IF(khz <= 0, -1, "bad input: khz=%f", khz);

  float divisor = 60000.0 / (three_phase ? khz * 1.5 : khz) / 2 - 1;
  int div = std::round(divisor);
  if (div < 0) div = 0;
  if (div > 0xffff) div = 0xffff;
  float actual_khz = 60000.0 / ((div + 1) * 2);
  if (three_phase) actual_khz = actual_khz / 3 * 2;
  float error = std::fabs(actual_khz - khz) / khz;
  std::printf("MPSSE requested %.02fkHz, div %d, actual %.02fkHz, error %.02f%%\n", khz,
              div, actual_khz, error * 100);

  int ret = 0;
  BufferClear();
  ret |= BufferByte(three_phase ? EN_3_PHASE : DIS_3_PHASE);
  ret |= BufferByte(adaptive ? EN_ADAPTIVE : DIS_ADAPTIVE);
  ret |= BufferByte(DIS_DIV_5); // disable div by 5 (60MHz)

  uint8_t cmd_tck_divisor[] = {TCK_DIVISOR,                              // set clock divisor
                               static_cast<uint8_t>(div & 0xff),         // div low
                               static_cast<uint8_t>((div >> 8) & 0xff)}; // div high
  ret |= BufferBytes(cmd_tck_divisor);
  ret |= BufferFlush();
  return ret;
}

int FtdiDevice::MpsseSetLowerPins(uint8_t state, uint8_t dir) {
  uint8_t cmd[] = {SET_BITS_LOW, state, dir};

  int err = 0;
  err = BufferBytes(cmd);
  if (err) return -1;
  err = BufferFlush();
  if (err) return -1;
  return 0;
}

std::unique_ptr<MpsseI2c> MpsseI2c::Create(FtdiDevice *dev, float scl_khz) {
  int err = ftdi_set_bitmode(dev->context(), 0xff, BITMODE_MPSSE);
  RETURN_IF(err != 0, nullptr, "ftdi_set_bitmode() failed: %d", err);
  // Use the desctructor to cleanup the bitmode setting.
  auto ret = std::unique_ptr<MpsseI2c>(new MpsseI2c(dev));

  err = dev->MpsseSync();
  RETURN_IF(err != 0, nullptr, "MpsseSync() failed: %d", err);

  err = dev->MpsseSetClockFreq(scl_khz, /*three_phase=*/true, /*adaptive=*/false);
  RETURN_IF(err != 0, nullptr, "MpsseSetClockFreq() failed: %d", err);

  err = ret->InitializePins();
  RETURN_IF(err != 0, nullptr, "InitializeI2cPins() failed: %d", err);

  return ret;
}

MpsseI2c::~MpsseI2c() {
  int ret = ftdi_set_bitmode(dev_->context(), 0xff, BITMODE_RESET);
  if (ret != 0) {
    std::fprintf(stderr, "ftdi_set_bitmode() reset failed: %d\n", ret);
  }
}

// Postcond: SDA & SCL both hold high.
int MpsseI2c::InitializePins() {
  dev_->BufferClear();
  return dev_->MpsseSetLowerPins(
    /*state=*/ 0b0000'0011,
    /*dir=*/   0b0000'0011
  );
}

int MpsseI2c::Start() {
  // First set SDA to LOW, indicates start
  if (dev_->MpsseSetLowerPins(
    0b00000001,
    0b00000011
  )) return -1;

  // Then bring SCL to low prepare for data tx, time gap is needed.
  // Time gap is established by two separate write calls.
  if (dev_->MpsseSetLowerPins(
    0b00000000,
    0b00000011
  )) return -1;

  return 0;
}

int MpsseI2c::Restart() {
  // Time gap is needed at all places.
  if (dev_->MpsseSetLowerPins(0b00000010, 0b00000011)) return -1;
  if (dev_->MpsseSetLowerPins(0b00000011, 0b00000011)) return -1;
  if (dev_->MpsseSetLowerPins(0b00000001, 0b00000011)) return -1;
  if (dev_->MpsseSetLowerPins(0b00000000, 0b00000011)) return -1;
  return 0;
}

int MpsseI2c::Stop() {
  // Time gap is needed at all places.
  if (dev_->MpsseSetLowerPins(0b00000001, 0b00000011)) return -1;
  if (dev_->MpsseSetLowerPins(0b00000011, 0b00000011)) return -1;
  return 0;
}

// MPSSE data TX clock edge limitation:
//
// In MPSSE, you have the option to specify MPSSE_WRITE_NEG or MPSSE_READ_NEG.
// However, you must use the correct one depending on the idle state of the clock.
// Otherwise the MPSSE will misbehave.
//
// If clock idle at ... you must use
// LOW                  WRITE_NEG  or  READ_POS
// HIGH                 WRITE_POS  or  READ_NEG
//
//                         2-phase-clk      3-phase-clk
// clk-idle-low            __/‾‾\__/‾‾\    __/‾‾\_____/‾‾\__
// data-write-neg          <=1=> <=2=>     <=1====> <=2====>
//
// clk-idle-high           ‾‾\__/‾‾\__/    ‾‾\__/‾‾‾‾‾\__/‾‾
// data-write-pos          <=1=> <=2=>     <=1====> <=2====>
//
// e.g. For I2C, the TX data must be stable when clock is high, so we have to use idle-low
//      clocking, and should always use WRITE_NEG and never use READ_NEG.
#define MPSSE_IDLE_LOW_WRITE  (MPSSE_DO_WRITE | MPSSE_WRITE_NEG)
#define MPSSE_IDLE_HIGH_WRITE (MPSSE_DO_WRITE)
#define MPSSE_IDLE_LOW_READ   (MPSSE_DO_READ)
#define MPSSE_IDLE_HIGH_READ  (MPSSE_DO_READ | MPSSE_READ_NEG)

int MpsseI2c::WriteByte(uint8_t data) {
  uint8_t cmds[] = {
    // Transfer 8 bits.
    MPSSE_IDLE_LOW_WRITE | MPSSE_BITMODE,
    0x7,  // 0x7 == 8 bits
    data,  // the byte

    // Both SDA and SCL should be LOW now, set ADBUS1 to INPUT mode so ADBUS2 can read the ack.
    // Time gap is not needed since the write should hold the data for 1/3 cycle after the pulse.
    SET_BITS_LOW,
    0b00000000,
    0b00000001,

    // Read ACK bit
    // Time gap is not needed before nor after because the clock should extend 1/3 cycle each direction.
    MPSSE_IDLE_LOW_READ | MPSSE_BITMODE,
    0,  // 0 = 1bit

    // Ask device to flush data back to PC, so the ftdi_read_byte below can be fast.
    SEND_IMMEDIATE,

    // Immediately take back the control of the SDA line and hold it low.
    // This step can in theory be postponsed and be done before the next write, or omitted if an i2c read follows.
    // But for simplicity of the reasoning about the pre/post cond, it's left here.
    SET_BITS_LOW,
    0b00000000,
    0b00000011
  };

  int err = 0;
  err = dev_->BufferBytes(cmds); if (err) return -1;
  err = dev_->BufferFlush(); if (err) return -1;

  uint8_t ack_bit;
  err = dev_->Read(&ack_bit, 1);
  if (err) return -1;

  // Low is ACK, high is NACK
  return (ack_bit & 0x1) ? 0 : 1;
}

int MpsseI2c::ReadBytes(uint16_t len, void* buf) {
  int err = 0;
  if (len == 0) return 0;
  // All operations can be done continuously without time gap in between.
  for (int i = 0; i < len; ++i) {
    // Release SDA line for reading.
    uint8_t rel_sda[] = {SET_BITS_LOW, 0b00000000, 0b00000001};
    err = dev_->BufferBytes(rel_sda); if (err) return -1;

    // READ 1 byte, 0x7=8bits
    uint8_t read_cmd[] = {MPSSE_IDLE_LOW_READ | MPSSE_BITMODE , 0x7};
    err = dev_->BufferBytes(read_cmd); if (err) return -1;

    // Re-acquire SDA
    uint8_t acq_sda[] = {SET_BITS_LOW, 0b00000000, 0b00000011};
    err = dev_->BufferBytes(acq_sda); if (err) return -1;

    // Clock out the ACK or NACK.
    // Note for I2C, high(1) is NACK.
    // Also use MPSSE_LSB so the bit is taken from LSB, otherwise need to use 0x80.
    uint8_t send_ack[] = {
      MPSSE_IDLE_LOW_WRITE | MPSSE_BITMODE | MPSSE_LSB, 0, static_cast<uint8_t>((i == len-1) ? 1 : 0)};
    err = dev_->BufferBytes(send_ack); if (err) return -1;
  }
  // Flush all data to PC.
  err = dev_->BufferByte(SEND_IMMEDIATE); if (err) return -1;
  // Execute
  err = dev_->BufferFlush(); if (err) return -1;
  err = dev_->Read(buf, len); if (err) return -1;
  return 0;
}

int MpsseI2c::Transaction(uint8_t addr7,
                          const uint8_t* tx_data, int tx_len,
                          void* rx_buf, int rx_len) {
  RETURN_IF(tx_len < 0 || rx_len < 0, -1, "invalid argument");

  Start();
  std::unique_ptr<MpsseI2c, std::function<void(MpsseI2c*)>> stop_on_exit(this, [](MpsseI2c *obj){
    // Make sure we issue the stop sequence when we return.
    obj->Stop();
  });

  if (tx_len > 0) {
    int ack = 0;
    ack = WriteByte(Addr7ToData(addr7, /*read=*/false));
    if (ack < 0) return -1;
    if (ack == 0) return -2;
    for (int i = 0; i < tx_len; ++i) {
      ack = WriteByte(tx_data[i]);
      if (ack < 0) return -1;
      if (ack == 0) return -2;
    }

    if (rx_len == 0) return 0;  // No read. Issue Stop and return.
    Restart();  // Issue a restart to prepare the the Read.
  }

  int ack = 0;
  ack = WriteByte(Addr7ToData(addr7, /*read=*/true));
  if (ack < 0) return -1;
  if (ack == 0) return -2;

  if (rx_len == 0) return 0;

  int err = ReadBytes(rx_len, rx_buf);
  return err ? -1 : 0;
}

} // namespace mpsse_protocol
