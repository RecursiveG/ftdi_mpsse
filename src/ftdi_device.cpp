#include "mpsse_protocol.h"

namespace mpsse_protocol {

#define RETURN_IF(cond, ret, fmt, ...)                                                   \
  do {                                                                                   \
    if (cond) {                                                                          \
      std::fprintf(stderr, fmt "\n", ##__VA_ARGS__);                                     \
      return (ret);                                                                      \
    }                                                                                    \
  } while (0)

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
int FtdiDevice::BufferBytes(std::span<const uint8_t> data) {
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

int FtdiDevice::WaitTransmitterEmpty(uint32_t timeout_ms) {
  // Transmitter empty is bit 6 of the higher byte.
  constexpr uint16_t TEMT_MASK = 0x4000;
  int err;
  uint16_t status;

  if (timeout_ms == 0) {
    err = ftdi_poll_modem_status(context_, &status);
    if (err) return -1;
    if (status & TEMT_MASK) return 0;
  }

  for (int i = 0; i < timeout_ms; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    err = ftdi_poll_modem_status(context_, &status);
    if (err) return -1;
    if (status & TEMT_MASK) return 0;
  }

  std::fprintf(stderr, "Transmission didn't finish in %dms\n", timeout_ms);
  return -2;
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

}
