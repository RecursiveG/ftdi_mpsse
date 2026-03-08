#include "mpsse_protocol.h"

#include <cstdint>
#include <cstdio>
#include <ftdi.h>
#include <memory>

#define RETURN_IF(cond, ret, fmt, ...)                                                                  \
  do {                                                                                                  \
    if (cond) {                                                                                         \
      std::fprintf(stderr, fmt "\n", ##__VA_ARGS__);                                                    \
      return (ret);                                                                                     \
    }                                                                                                   \
  } while (0)

#define RETURN_IF_ERR(st)                                                                               \
  do {                                                                                                  \
    Status s = (st);                                                                                    \
    if (!s.ok()) return s;                                                                              \
  } while (0)

namespace mpsse_protocol {

std::unique_ptr<MpsseWs2812b> MpsseWs2812b::Create(FtdiDevice *dev) {
  int err = ftdi_set_bitmode(dev->context(), 0xff, BITMODE_MPSSE);
  RETURN_IF(err != 0, nullptr, "ftdi_set_bitmode() failed: %d", err);
  // Use the desctructor to cleanup the bitmode setting.
  auto ret = std::unique_ptr<MpsseWs2812b>(new MpsseWs2812b(dev));

  Status st = dev->MpsseSync();
  RETURN_IF(!st.ok(), nullptr, "MpsseSync() failed: %s", st.human().c_str());

  st = dev->MpsseSetClockFreq(2500, /*three_phase=*/false, /*adaptive=*/false);
  RETURN_IF(!st.ok(), nullptr, "MpsseSetClockFreq() failed: %s", st.human().c_str());

  dev->BufferClear();
  st = dev->MpsseSetLowerPins(
    /*state=*/ 0b0000'0000,  // idle-low clock.
    /*dir=*/   0b0000'0011
  );
  RETURN_IF(!st.ok(), nullptr, "MpsseSetLowerPins() failed: %s", st.human().c_str());

  return ret;
}

MpsseWs2812b::~MpsseWs2812b() {
  dev_->WaitTransmitterEmpty();
  int ret = ftdi_set_bitmode(dev_->context(), 0xff, BITMODE_RESET);
  if (ret != 0) {
    std::fprintf(stderr, "ftdi_set_bitmode() reset failed: %d\n", ret);
  }
}

void MpsseWs2812b::ExpandByte(uint8_t byte, uint8_t buf[]) {
  buf[0] = 0;
  buf[1] = 0;
  buf[2] = 0;
  buf[0] |= (byte & 0x80) ? 0xc0 : 0x80;
  buf[0] |= (byte & 0x40) ? 0x18 : 0x10;
  buf[0] |= (byte & 0x20) ? 0x03 : 0x02;
  buf[1] |= (byte & 0x10) ? 0x60 : 0x40;
  buf[1] |= (byte & 0x08) ? 0x0d : 0x09;
  buf[2] |= (byte & 0x04) ? 0x80 : 0x00;
  buf[2] |= (byte & 0x02) ? 0x30 : 0x20;
  buf[2] |= (byte & 0x01) ? 0x06 : 0x04;
}

Status MpsseWs2812b::SendFrame(std::span<const uint32_t> rgb) {
  auto raw = std::make_unique<uint8_t[]>(rgb.size() * 9);
  // WS2812B wants the 3 bytes in GRB order.
  for (size_t i = 0; i < rgb.size() ; i++) {
    ExpandByte((rgb[i] >> 16) & 0xff, raw.get() + i*9 + 3);
    ExpandByte((rgb[i] >> 8 ) & 0xff, raw.get() + i*9    );
    ExpandByte((rgb[i]      ) & 0xff, raw.get() + i*9 + 6);
  }
  return SendRaw({raw.get(), rgb.size()*9});
}

Status MpsseWs2812b::SendRaw(std::span<const uint8_t> raw) {
  // Header for writing bytes.
  RETURN_IF_ERR(dev_->BufferBytes({
    MPSSE_IDLE_LOW_WRITE,
    static_cast<uint8_t>( (raw.size()-1)       & 0xff),
    static_cast<uint8_t>(((raw.size()-1) >> 8) & 0xff)
  }));
  RETURN_IF_ERR(dev_->BufferFlush());

  // Because the data array can be huge, we use unbuffered write.
  RETURN_IF_ERR(dev_->Write(raw.data(), raw.size()));

  // Last bit must be zero. Clock 128bits to signal a reset.
  RETURN_IF_ERR(dev_->BufferBytes({CLK_BYTES, 16, 0}));
  RETURN_IF_ERR(dev_->BufferFlush());

  return Status::Ok();
}

} // namespace mpsse_protocol
