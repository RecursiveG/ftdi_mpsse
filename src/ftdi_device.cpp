#include "mpsse_protocol.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <format>
#include <ftdi.h>
#include <initializer_list>
#include <memory>
#include <span>
#include <thread>

namespace mpsse_protocol {

std::unique_ptr<FtdiDevice> FtdiDevice::OpenVendorProduct(uint16_t id_vendor, uint16_t id_product,
                                                          enum ftdi_interface intf) {
  int err = 0;
  struct ftdi_context *ctx = nullptr;

  ctx = ftdi_new();
  if (ctx == nullptr) {
    std::fprintf(stderr, "ftdi_new() failed\n");
    return nullptr;
  }
  err = ftdi_set_interface(ctx, intf);
  if (err) {
    ftdi_free(ctx);
    std::fprintf(stderr, "ftdi_set_interface() failed: %d\n", err);
    return nullptr;
  }
  err = ftdi_usb_open(ctx, id_vendor, id_product);
  if (err) {
    ftdi_free(ctx);
    std::fprintf(stderr, "ftdi_usb_open() failed: %d\n", err);
    return nullptr;
  }
  return std::make_unique<FtdiDevice>(ctx);
}

std::unique_ptr<FtdiDevice> FtdiDevice::OpenBusDevice(int bus, int device, enum ftdi_interface intf) {
  int err = 0;
  struct ftdi_context *ctx = nullptr;

  ctx = ftdi_new();
  if (ctx == nullptr) {
    std::fprintf(stderr, "ftdi_new() failed\n");
    return nullptr;
  }
  err = ftdi_set_interface(ctx, intf);
  if (err) {
    ftdi_free(ctx);
    std::fprintf(stderr, "ftdi_set_interface() failed: %d\n", err);
    return nullptr;
  }
  err = ftdi_usb_open_bus_addr(ctx, bus, device);
  if (err) {
    ftdi_free(ctx);
    std::fprintf(stderr, "ftdi_usb_open_bus_addr() failed: %d\n", err);
    return nullptr;
  }

  return std::make_unique<FtdiDevice>(ctx);
}

void FtdiDevice::FreeContext(struct ftdi_context *context) {
  if (context) {
    ftdi_free(context);
  }
}

Status FtdiDevice::BufferByte(uint8_t data) {
  if (buffer_used_ >= kBufferSize) {
    return Status::Err("Too many data in buffer");
  }
  buffer_[buffer_used_++] = data;
  return Status::Ok();
}

Status FtdiDevice::BufferBytes(std::span<const uint8_t> data) {
  if (buffer_used_ + data.size() > kBufferSize) {
    return Status::Err("Too many bytes to write");
  }
  for (ssize_t i = 0; i < data.size(); i++) {
    buffer_[buffer_used_++] = data[i];
  }
  return Status::Ok();
}

Status FtdiDevice::BufferBytes(std::initializer_list<uint8_t> data) {
  if (buffer_used_ + data.size() > kBufferSize) {
    return Status::Err("Too many bytes to write");
  }
  for (uint8_t x : data) {
    buffer_[buffer_used_++] = x;
  }
  return Status::Ok();
}

Status FtdiDevice::BufferFlush() {
  if (buffer_used_ == 0) {
    return Status::Ok();
  }
  int ret = ftdi_write_data(context_.get(), buffer_, buffer_used_);
  if (ret != buffer_used_) {
    return Status::Err(std::format("ftdi_write_data() failed: expected {} got {}", buffer_used_, ret));
  }
  buffer_used_ = 0;
  return Status::Ok();
}

Status FtdiDevice::Write(const void *buf, int32_t len) {
  if (len < 0) return Status::Err("Invalid length");
  if (buffer_used_ > 0) return Status::Err("Attempt to write but buffer is not empty.");

  int ret = ftdi_write_data(context_.get(), static_cast<const uint8_t *>(buf), len);
  if (ret != len) {
    return Status::Err(std::format("ftdi_write_data() failed: expected {} got {}", len, ret));
  }
  return Status::Ok();
}

Status FtdiDevice::Read(void *buf, int32_t len, std::chrono::duration<double> timeout) {
  auto begin = std::chrono::high_resolution_clock::now();
  auto deadline = begin + timeout;
  auto *ptr = static_cast<unsigned char *>(buf);
  do {
    int read = ftdi_read_data(context_.get(), static_cast<unsigned char *>(ptr), len);
    if (read == 0) continue;
    if (read < 0) {
      return Status::Err(std::format("ftdi_read_bytes() failed: {}", read));
    }
    len -= read;
    if (read < 0) {
      return Status::Err(std::format("ftdi_read_bytes() got too many bytes: {}", len));
    }
    if (len == 0) {
      return Status::Ok();
    }
    ptr += read;
  } while (std::chrono::high_resolution_clock::now() < deadline);
  return Status::Err("ftdi_read_bytes() timed out");
}

Status FtdiDevice::WaitTransmitterEmpty(uint32_t timeout_ms) {
  // Transmitter empty is bit 6 of the higher byte.
  constexpr uint16_t TEMT_MASK = 0x4000;
  int err;
  uint16_t status;

  if (timeout_ms == 0) {
    err = ftdi_poll_modem_status(context_.get(), &status);
    if (err) return Status::Err(std::format("ftdi_pool_modem_status() failed: {}", err));
    if (status & TEMT_MASK) {
      return Status::Ok();
    }
  }

  for (int i = 0; i < timeout_ms; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    err = ftdi_poll_modem_status(context_.get(), &status);
    if (err) return Status::Err(std::format("ftdi_pool_modem_status() failed: {}", err));
    if (status & TEMT_MASK) {
      return Status::Ok();
    }
  }

  return Status::Err("Transmission didn't finish in time.");
}

Status FtdiDevice::MpsseSync() {
  constexpr int kBufSize = 256;
  uint8_t out_data[] = {0xab, 0xaa}; // two bad commands
  uint8_t buf[kBufSize];
  uint32_t in_data = 0;
  int ret;

  // Write two commands, expect echo back.
  ret = ftdi_write_data(context_.get(), out_data, 2);
  if (ret != 2) return Status::Err(std::format("ftdi_write_data() failed: {}", ret));

  // Spin for at least 100us, if data not ready, spin up to 10ms.
  auto begin = std::chrono::high_resolution_clock::now();
  auto GetElapsedUs = [&begin]() -> uint64_t {
    return std::chrono::duration<double, std::micro>(std::chrono::high_resolution_clock::now() - begin)
        .count();
  };
  while (true) {
    uint64_t elapsed = GetElapsedUs();
    if (elapsed > 10'000) break;
    if (elapsed > 100 && in_data == 0xfaabfaaa) break;

    ret = ftdi_read_data(context_.get(), buf, kBufSize);
    if (ret < 0) return Status::Err(std::format("ftdi_read_data() failed: {}", ret));
    if (ret > 0) {
      // TODO: maybe bug around fmin?
      for (int i = 0; i < std::fmin(ret, 4); i++) {
        // Sliding window looking for the last 4 bytes returned.
        in_data = (in_data << 8) | buf[i];
      }
    }
  }
  if (in_data != 0xfaabfaaa) {
    return Status::Err("MPSSE synchronization failed");
  }
  return Status::Ok();
}

Status FtdiDevice::MpsseSetClockFreq(float khz, bool three_phase, bool adaptive) {
  if (khz <= 0) return Status::Err("Invalid khz input");

  float divisor = 60000.0 / (three_phase ? khz * 1.5 : khz) / 2 - 1;
  int div = std::round(divisor);
  if (div < 0) div = 0;
  if (div > 0xffff) div = 0xffff;
  float actual_khz = 60000.0 / ((div + 1) * 2);
  if (three_phase) actual_khz = actual_khz / 3 * 2;
  float error = std::fabs(actual_khz - khz) / khz;
  std::printf("MPSSE requested %.02fkHz, div %d, actual %.02fkHz, error %.02f%%\n", khz, div, actual_khz,
              error * 100);

  Status ret = Status::Ok();
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

Status FtdiDevice::MpsseSetLowerPins(uint8_t state, uint8_t dir, bool flush) {
  // Only change the low 4 bits.
  low_pin_state_ = (low_pin_state_ & 0xf0) | (state & 0x0f);
  low_pin_dir_ = (low_pin_dir_ & 0xf0) | (dir & 0x0f);
  uint8_t cmd[] = {SET_BITS_LOW, low_pin_state_, low_pin_dir_};

  auto st = BufferBytes(cmd);
  if (!st.ok()) return st;
  if (flush) return BufferFlush();
  return Status::Ok();
}

Status FtdiDevice::GpioSetLowerPins(uint8_t state, uint8_t dir, bool flush) {
  // Only change the high 4 bits.
  low_pin_state_ = (low_pin_state_ & 0x0f) | (state & 0xf0);
  low_pin_dir_ = (low_pin_dir_ & 0x0f) | (dir & 0xf0);
  uint8_t cmd[] = {SET_BITS_LOW, low_pin_state_, low_pin_dir_};

  auto st = BufferBytes(cmd);
  if (!st.ok()) return st;
  if (flush) return BufferFlush();
  return Status::Ok();
}

} // namespace mpsse_protocol
