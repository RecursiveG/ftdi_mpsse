#include "mpsse_protocol.h"

namespace mpsse_protocol {

std::unique_ptr<MpsseWs2812b> MpsseWs2812b::Create(FtdiDevice *dev) {
  int err = ftdi_set_bitmode(dev->context(), 0xff, BITMODE_MPSSE);
  RETURN_IF(err != 0, nullptr, "ftdi_set_bitmode() failed: %d", err);
  // Use the desctructor to cleanup the bitmode setting.
  auto ret = std::unique_ptr<MpsseWs2812b>(new MpsseWs2812b(dev));

  err = dev->MpsseSync();
  RETURN_IF(err != 0, nullptr, "MpsseSync() failed: %d", err);

  err = dev->MpsseSetClockFreq(2500, /*three_phase=*/false, /*adaptive=*/false);
  RETURN_IF(err != 0, nullptr, "MpsseSetClockFreq() failed: %d", err);

  dev->BufferClear();
  err = dev->MpsseSetLowerPins(
    /*state=*/ 0b0000'0000,  // idle-low clock.
    /*dir=*/   0b0000'0011
  );
  RETURN_IF(err != 0, nullptr, "MpsseSetLowerPins() failed: %d", err);

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

int MpsseWs2812b::SendFrame(std::span<const uint32_t> rgb) {
  auto raw = std::make_unique<uint8_t[]>(rgb.size() * 9);
  // WS2812B wants the 3 bytes in GRB order.
  for (size_t i = 0; i < rgb.size() ; i++) {
    ExpandByte((rgb[i] >> 16) & 0xff, raw.get() + i*9 + 3);
    ExpandByte((rgb[i] >> 8 ) & 0xff, raw.get() + i*9    );
    ExpandByte((rgb[i]      ) & 0xff, raw.get() + i*9 + 6);
  }
  return SendRaw({raw.get(), rgb.size()*9});
}

int MpsseWs2812b::SendRaw(std::span<const uint8_t> raw) {
  uint8_t header[3];
  header[0] = MPSSE_IDLE_LOW_WRITE;
  header[1] = (raw.size()-1) & 0xff;
  header[2] = ((raw.size()-1) >> 8) & 0xff;
  // Last bit must be zero. Clock 128bits to signal a reset.
  uint8_t resetcmd[] = {CLK_BYTES, 16, 0};

  int err = 0;
  err = dev_->BufferBytes(header); if (err) return -1;
  err = dev_->BufferFlush(); if (err) return -1;

  // Because the data array can be huge, we use ftdi_write_data() directly.
  err = ftdi_write_data(dev_->context(), raw.data(), raw.size());
  RETURN_IF(err != raw.size(), -1, "ftdi_write_data() failed: expect %zu got %d", raw.size(), err);

  err = dev_->BufferBytes(resetcmd); if (err) return -1;
  err = dev_->BufferFlush(); if (err) return -1;
  return 0;
}

}
