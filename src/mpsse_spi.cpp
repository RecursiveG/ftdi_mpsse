#include "mpsse_protocol.h"

namespace mpsse_protocol {

// SPI Modes
// Mode 0: Use normal two phase CLOCK, WRITE_NEG, READ_POS
//  CLK    __/‾‾\__/‾‾\          __/‾‾\__/‾‾\_
// MOSI    <===> <===>   MISO    <===> <===>
//
// Mode 1: Cannot use two phase clock. WRITE_POS is invalid when clock is idle low.
//         Use three phase clock to mimic, we need to read one more bit and discard the first read.
// (BAD)  CLK    __/‾‾\__/‾‾\_
// (BAD) MOSI       <===> <===>
//
//  CLK          __/‾‾\_____/‾‾\__       __/‾‾\_____/‾‾\_____/‾‾\__
// MOSI          <====^=> <====^=>  MISO XX <======> <======> XXXXX
//                                        ^-bogus  ^-bit0   ^-bit1

std::unique_ptr<MpsseSpi> MpsseSpi::Create(FtdiDevice *dev, int cpol, int cpha, float clk_mhz) {
  int err = ftdi_set_bitmode(dev->context(), 0xff, BITMODE_MPSSE);
  RETURN_IF(err != 0, nullptr, "ftdi_set_bitmode() failed: %d", err);
  // Use the desctructor to cleanup the bitmode setting.
  auto ret = std::unique_ptr<MpsseSpi>(new MpsseSpi(dev, cpol, cpha));

  err = dev->MpsseSync();
  RETURN_IF(err != 0, nullptr, "MpsseSync() failed: %d", err);

  err = dev->MpsseSetClockFreq(clk_mhz * 1000, /*three_phase=*/cpha == 1, /*adaptive=*/false);
  RETURN_IF(err != 0, nullptr, "MpsseSetClockFreq() failed: %d", err);

  dev->BufferClear();
  err = dev->MpsseSetLowerPins(
    /*state=*/ 0b0000'1000 | (cpol & 1),  // clock idle based on cpol, CS is active low so we held it high.
    /*dir=*/   0b0000'1011
  );
  RETURN_IF(err != 0, nullptr, "MpsseSetLowerPins() failed: %d", err);
  return ret;
}

int MpsseSpi::Transaction(const void* tx_data, int tx_len, void* rx_data, int rx_len) {
  int err;
  uint8_t cmds[3];
  RETURN_IF(tx_len == 0 && rx_len == 0, -1, "tx & rx len cannot be both zero.");

  // pull down CS
  cmds[0] = SET_BITS_LOW;
  cmds[1] = 0b0000'0000 | (cpol_ & 1);
  cmds[2] = 0b0000'1011;
  err = dev_->BufferBytes(cmds); if (err) return -1;

  // Transmit cmd
  if (tx_len > 0) {
    cmds[0] = (cpol_ ? MPSSE_IDLE_HIGH_WRITE : MPSSE_IDLE_LOW_WRITE);
    cmds[1] = (tx_len-1) & 0xff;
    cmds[2] = ((tx_len-1) >> 8) & 0xff;
    err = dev_->BufferBytes(cmds); if (err) return -1;
    err = dev_->BufferBytes(std::span(static_cast<const uint8_t*>(tx_data), tx_len)); if (err) return -1;
  }

  // Read cmd
  if (rx_len > 0) {
    if (cpha_ == 1) {  // Read extra bit if cpha == 1
      cmds[0] = (cpol_ ? MPSSE_IDLE_HIGH_READ : MPSSE_IDLE_LOW_READ) | MPSSE_BITMODE;
      cmds[1] = 0;
      err = dev_->BufferBytes(std::span(cmds, 2)); if (err) return -1;
    }

    cmds[0] = (cpol_ ? MPSSE_IDLE_HIGH_READ : MPSSE_IDLE_LOW_READ);
    cmds[1] = (rx_len-1) & 0xff;
    cmds[2] = ((rx_len-1) >> 8) & 0xff;
    err = dev_->BufferBytes(cmds); if (err) return -1;

    // Flush
    err = dev_->BufferByte(SEND_IMMEDIATE); if (err) return -1;
  }

  // pull up CS
  cmds[0] = SET_BITS_LOW;
  cmds[1] = 0b0000'1000 | (cpol_ & 1);
  cmds[2] = 0b0000'1011;
  err = dev_->BufferBytes(cmds); if (err) return -1;

  // Issue the commands
  err = dev_->BufferFlush(); if (err) return -1;

  // Read out data.
  if (rx_len > 0) {
    if (cpha_ == 1) {  // discard the extra bit (in a standalone byte)
      uint8_t bogus;
      err = dev_->Read(&bogus, 1); if (err) return -1;
    }
    err = dev_->Read(rx_data, rx_len); if (err) return -1;
  }

  return 0;
}

MpsseSpi::~MpsseSpi() {
  // Make sure we pull up CS before leaving.
  dev_->BufferClear();
  dev_->MpsseSetLowerPins(
    /*state=*/ 0b0000'1000 | (cpol_ & 1),
    /*dir=*/   0b0000'1011);

  dev_->WaitTransmitterEmpty();
  int ret = ftdi_set_bitmode(dev_->context(), 0xff, BITMODE_RESET);
  if (ret != 0) {
    std::fprintf(stderr, "ftdi_set_bitmode() reset failed: %d\n", ret);
  }
}

} // namespace mpsse_protocol
