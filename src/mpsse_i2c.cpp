#include "mpsse_protocol.h"

#include <memory>
#include <functional>

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

std::unique_ptr<MpsseI2c> MpsseI2c::Create(FtdiDevice *dev, float scl_khz) {
  int err = ftdi_set_bitmode(dev->context(), 0xff, BITMODE_MPSSE);
  RETURN_IF(err != 0, nullptr, "ftdi_set_bitmode() failed: %d", err);
  // Use the desctructor to cleanup the bitmode setting.
  auto ret = std::unique_ptr<MpsseI2c>(new MpsseI2c(dev));

  Status st = dev->MpsseSync();
  RETURN_IF(!st.ok(), nullptr, "MpsseSync() failed: %s", st.human().c_str());

  st = dev->MpsseSetClockFreq(scl_khz, /*three_phase=*/true, /*adaptive=*/false);
  RETURN_IF(!st.ok(), nullptr, "MpsseSetClockFreq() failed: %s", st.human().c_str());

  // Initialize pin
  // Postcond: SDA & SCL both hold high.
  dev->BufferClear();
  st = dev->MpsseSetLowerPins(
    /*state=*/ 0b0000'0011,
    /*dir=*/   0b0000'0011
  );
  RETURN_IF(!st.ok(), nullptr, "InitializeI2cPins() failed: %s", st.human().c_str());

  return ret;
}

MpsseI2c::~MpsseI2c() {
  dev_->WaitTransmitterEmpty();
  int ret = ftdi_set_bitmode(dev_->context(), 0xff, BITMODE_RESET);
  if (ret != 0) {
    std::fprintf(stderr, "ftdi_set_bitmode() reset failed: %d\n", ret);
  }
}

// clang-format off
Status MpsseI2c::Start() {
  // First set SDA to LOW, indicates start
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(
    0b00000001,
    0b00000011
  ));

  // Then bring SCL to low prepare for data tx, time gap is needed.
  // Time gap is established by two separate write calls.
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(
    0b00000000,
    0b00000011
  ));

  return Status::Ok();
}
// clang-format on

Status MpsseI2c::Restart() {
  // Time gap is needed at all places.
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(0b00000010, 0b00000011));
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(0b00000011, 0b00000011));
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(0b00000001, 0b00000011));
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(0b00000000, 0b00000011));
  return Status::Ok();
}

Status MpsseI2c::Stop() {
  // Time gap is needed at all places.
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(0b00000001, 0b00000011));
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(0b00000011, 0b00000011));
  return Status::Ok();
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

Status MpsseI2c::WriteByte(uint8_t data, bool *ack) {
  uint8_t cmd_write_byte[] = {
    // Transfer 8 bits.
    MPSSE_IDLE_LOW_WRITE | MPSSE_BITMODE,
    0x7,  // 0x7 == 8 bits
    data,  // the byte
  };
  RETURN_IF_ERR(dev_->BufferBytes(cmd_write_byte));
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(
    // Both SDA and SCL should be LOW now, set ADBUS1 to INPUT mode so ADBUS2 can read the ack.
    // Time gap is not needed since the write should hold the data for 1/3 cycle after the pulse.
    0b00000000,
    0b00000001,
    /*flush=*/ false
  ));
  uint8_t cmd_read_bit[] = {
    // Read ACK bit
    // Time gap is not needed before nor after because the clock should extend 1/3 cycle each direction.
    MPSSE_IDLE_LOW_READ | MPSSE_BITMODE,
    0,  // 0 = 1bit
  };
  RETURN_IF_ERR(dev_->BufferBytes(cmd_read_bit));
  RETURN_IF_ERR(dev_->BufferByte(
    // Ask device to flush data back to PC, so the ftdi_read_byte below can be fast.
    SEND_IMMEDIATE
  ));
  RETURN_IF_ERR(dev_->MpsseSetLowerPins(
    // Immediately take back the control of the SDA line and hold it low.
    // This step can in theory be postponsed and be done before the next write, or omitted if an i2c read follows.
    // But for simplicity of the reasoning about the pre/post cond, it's left here.
    0b00000000,
    0b00000011,
    /*flush=*/ false
  ));

  RETURN_IF_ERR(dev_->BufferFlush());

  uint8_t ack_bit;
  RETURN_IF_ERR(dev_->Read(&ack_bit, 1));

  if (ack) {
    // Low is ACK, high is NACK
    *ack = (ack_bit & 0x1) == 0;
  }

  return Status::Ok();
}

Status MpsseI2c::ReadBytes(uint16_t len, void* buf) {
  if (len > 320) return Status::Err("Too many data to read");
  if (len == 0) return Status::Ok();
  // All operations can be done continuously without time gap in between.
  for (int i = 0; i < len; ++i) {
    // Release SDA line for reading.
    RETURN_IF_ERR(dev_->MpsseSetLowerPins(0b00000000, 0b00000001, false));

    // READ 1 byte, 0x7=8bits
    RETURN_IF_ERR(dev_->BufferBytes({MPSSE_IDLE_LOW_READ | MPSSE_BITMODE , 0x7}));

    // Re-acquire SDA
    RETURN_IF_ERR(dev_->MpsseSetLowerPins(0b00000000, 0b00000011, false));

    // Clock out the ACK or NACK.
    // Note for I2C, high(1) is NACK.
    // Also use MPSSE_LSB so the bit is taken from LSB, otherwise need to use 0x80.
    RETURN_IF_ERR(dev_->BufferBytes({
      MPSSE_IDLE_LOW_WRITE | MPSSE_BITMODE | MPSSE_LSB,
      0, // 0=1bit
      static_cast<uint8_t>((i == len-1) ? 1 : 0)}));
  }
  // Flush all data to PC.
  RETURN_IF_ERR(dev_->BufferByte(SEND_IMMEDIATE));
  // Execute
  RETURN_IF_ERR(dev_->BufferFlush());
  return dev_->Read(buf, len);
}

Status MpsseI2c::Transaction(uint8_t addr7,
                          const uint8_t* tx_data, int tx_len,
                          void* rx_buf, int rx_len) {
  if (tx_len < 0 || rx_len < 0) return Status::Err("Invalid arguments");

  RETURN_IF_ERR(Start());
  std::unique_ptr<MpsseI2c, std::function<void(MpsseI2c*)>> stop_on_exit(this, [](MpsseI2c *obj){
    // Make sure we issue the stop sequence when we return.
    auto st = obj->Stop();
    if (!st.ok()) {
      std::fprintf(stderr, "Failed to issue an stop for I2C transaction: %s\n", st.human().c_str());
    }
  });

  if (tx_len > 0) {
    // int ack = 0;
    bool ack = false;

    RETURN_IF_ERR(WriteByte(Addr7ToData(addr7, /*read=*/false), &ack));
    if (!ack) return Status::Err("No ack from device");
    for (int i = 0; i < tx_len; ++i) {
      RETURN_IF_ERR(WriteByte(tx_data[i], &ack));
      if (!ack) return Status::Err("NACK before all data sent");
    }

    if (rx_len == 0) return Status::Ok();  // No read. Issue Stop and return.
    RETURN_IF_ERR(Restart());  // Issue a restart to prepare the the Read.
  }

  bool ack = false;
  RETURN_IF_ERR(WriteByte(Addr7ToData(addr7, /*read=*/true), &ack));
  if (!ack) return Status::Err("No ack from device for read");

  if (rx_len == 0) return Status::Ok();

  return ReadBytes(rx_len, rx_buf);
}

}
