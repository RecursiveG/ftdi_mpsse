#include "mpsse_protocol.h"

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

std::unique_ptr<MpsseSpi> MpsseSpi::Create(FtdiDevice *dev, int cpol, int cpha, float clk_mhz) {
  int err = ftdi_set_bitmode(dev->context(), 0xff, BITMODE_MPSSE);
  RETURN_IF(err != 0, nullptr, "ftdi_set_bitmode() failed: %d", err);
  // Use the desctructor to cleanup the bitmode setting.
  auto ret = std::unique_ptr<MpsseSpi>(new MpsseSpi(dev, cpol, cpha));

  Status st = dev->MpsseSync();
  RETURN_IF(!st.ok(), nullptr, "MpsseSync() failed: %s", st.human().c_str());

  st = dev->MpsseSetClockFreq(clk_mhz * 1000, /*three_phase=*/cpha == 1, /*adaptive=*/false);
  RETURN_IF(!st.ok(), nullptr, "MpsseSetClockFreq() failed: %s", st.human().c_str());

  dev->BufferClear();
  st = dev->MpsseSetLowerPins(
    /*state=*/ 0b0000'1000 | (cpol & 1),  // clock idle based on cpol, CS is active low so we held it high.
    /*dir=*/   0b0000'1011
  );
  RETURN_IF(!st.ok(), nullptr, "MpsseSetLowerPins() failed: %s", st.human().c_str());
  return ret;
}

// TODO: revise all the flushes in this function.
Status MpsseSpi::Transaction(const void* tx_data, int tx_len, void* rx_data, int rx_len) {
  if (tx_len == 0 && rx_len == 0) return Status::Err("tx & rx len cannot be both zero.");

  // pull down CS
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(
    0b0000'0000 | (cpol_ & 1),
    0b0000'1011
  ));

  // Transmit cmd
  if (tx_len > 0) {
    RETURN_IF_ERR(dev_->BufferBytes({
      static_cast<uint8_t>(cpol_ ? MPSSE_IDLE_HIGH_WRITE : MPSSE_IDLE_LOW_WRITE),
      static_cast<uint8_t>((tx_len-1) & 0xff),
      static_cast<uint8_t>(((tx_len-1) >> 8) & 0xff),
    }));
    RETURN_IF_ERR(dev_->BufferFlush());
    RETURN_IF_ERR(dev_->Write(static_cast<const uint8_t*>(tx_data), tx_len));
  }

  // Read cmd
  if (rx_len > 0) {
    if (cpha_ == 1) {  // Read extra bit if cpha == 1
      RETURN_IF_ERR(dev_->BufferBytes({
        static_cast<uint8_t>((cpol_ ? MPSSE_IDLE_HIGH_READ : MPSSE_IDLE_LOW_READ) | MPSSE_BITMODE),
        0,
      }));
    }

    RETURN_IF_ERR(dev_->BufferBytes({
      static_cast<uint8_t>(cpol_ ? MPSSE_IDLE_HIGH_READ : MPSSE_IDLE_LOW_READ),
      static_cast<uint8_t>((rx_len-1) & 0xff),
      static_cast<uint8_t>(((rx_len-1) >> 8) & 0xff),
    }));

    // Flush
    RETURN_IF_ERR(dev_->BufferByte(SEND_IMMEDIATE));
  }

  // pull up CS
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(
    0b0000'1000 | (cpol_ & 1),
    0b0000'1011
  ));

  // Issue the commands
  RETURN_IF_ERR(dev_->BufferFlush());

  // Read out data.
  if (rx_len > 0) {
    if (cpha_ == 1) {  // discard the extra bit (in a standalone byte)
      uint8_t bogus;
      RETURN_IF_ERR(dev_->Read(&bogus, 1));
    }
    RETURN_IF_ERR(dev_->Read(rx_data, rx_len));
  }

  return Status::Ok();
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
