#ifndef __MPSSE_PROTOCOL_H__
#define __MPSSE_PROTOCOL_H__

#include <chrono>
#include <cstdint>
#include <cstring>
#include <format>
#include <ftdi.h>
#include <memory>
#include <span>
#include <string>
#include <initializer_list>

namespace mpsse_protocol {

class Status {
public:
  static Status Ok() { return {}; }
  static Status Err(std::string msg) { return {-1, msg}; }
  static Status Errno(int errno, std::string msg) { return {errno, msg}; }

  // If the status is OK.
  bool ok() const { return err_ == 0; }

  // Return the error code:
  //               0 : OK
  // positive number : Linux errno
  //              -1 : Other errors
  int err() const { return err_; }

  // Return the attached message.
  const std::string &msg() const { return msg_; }

  // Return a human readable message.
  std::string human() const {
    if (err_ == 0) return "OK";
    if (err_ == -1) return "Error: " + msg_;
    return std::format("Errno {} {}. {}", err_, strerror(err_), msg_);
  }

  // Quick and dirty way to chain multiple Status together
  Status &operator|=(const Status &rhs) {
    if (err_ == 0) {
      err_ = rhs.err_;
      msg_ = rhs.msg_;
    } else if (!rhs.ok()) {
      msg_ += " -> chained error " + rhs.human();
    }
    return *this;
  }

private:
  Status(int err = 0, std::string msg = "") : err_(err), msg_(msg) {}
  int err_ = 0;
  std::string msg_ = "";
};

class FtdiDevice {
public:
  static std::unique_ptr<FtdiDevice> OpenVendorProduct(uint16_t id_vendor, uint16_t id_product,
                                                       enum ftdi_interface intf = INTERFACE_ANY);
  // This is the device number. Not to be confused by the port number which may also shown
  // as "x-y"
  static std::unique_ptr<FtdiDevice> OpenBusDevice(int bus, int device,
                                                   enum ftdi_interface intf = INTERFACE_ANY);
  static void FreeContext(struct ftdi_context *context);

  // The context will be freed on destruction.
  FtdiDevice() : FtdiDevice(nullptr) {}
  explicit FtdiDevice(struct ftdi_context *context) : context_(context, &FreeContext) {}
  virtual ~FtdiDevice() = default;
  struct ftdi_context *context() { return context_.get(); }

  //
  // Helper functions used by different interface classes.
  //

  // First stash bytes into a buffer, then use BufferFlush() to flush all bytes to the
  // device. If there are not enough space in the buffer, it will return an error, and the
  // buffer is unchanged. Use the Write() function to avoid extra copy.
  static constexpr int kBufferSize = 4096;  // FT2232's internal buffer is 4K.
  void BufferClear() { buffer_used_ = 0; }
  Status BufferByte(uint8_t data);
  Status BufferBytes(std::span<const uint8_t> data);
  Status BufferBytes(std::initializer_list<uint8_t> data);
  // Buffer content is unchanged if Flush errors.
  Status BufferFlush();

  // Bypass the buffer and write directly to the device.
  // Return an error if the buffer is not empty.
  Status Write(const void *buf, int32_t len);

  // Wrapper around ftdi_read_data()
  Status Read(void *buf, int32_t len,
              std::chrono::duration<double> timeout = std::chrono::milliseconds(1));

  // Wait for "Transmitter empty" bit set. Return 0 if ok, -1 if error, -2 if timeout.
  Status WaitTransmitterEmpty(uint32_t timeout_ms = 1000);

  // Synchronize the MPSSE state, must be run before other MPSSE commands.
  // Note the implementation isn't identical to AN_135
  Status MpsseSync();
  Status MpsseSetClockFreq(float khz, bool three_phase, bool adaptive);

  // Helper functions for controlling the lower 8 pins.
  // Users SHOULD NOT use functions here and SHOULD use MpsseGpio class instead!
  //
  // This is convoluted because MPSSE doesn't support masking and must change
  // 8 pins at once. So the masking and memorization is done from software.
  // The lower 4 pins should generally controlled by the MPSSE.
  // The high 4 pins may be used by the user as GPIOs.

  // Append command to the existing buffer and optionally flush.
  // Buffer will be in an unspecified state if error happens.
  //
  // State:     1=high   0=low
  // Direction: 1=output 0=input
  // bit[x]:    ADBUSx
  // Mask: which pins should be changed
  Status MpsseUpdateLowerPins(uint8_t state, uint8_t dir, uint8_t mask, bool flush);
  // Shortcut for controlling the lower 4 bits.
  Status MpsseSetLowerPins(uint8_t state, uint8_t dir) {
    return MpsseUpdateLowerPins(state, dir, 0xf, /*flush=*/true);
  }
  Status MpsseBufferLowerPins(uint8_t state, uint8_t dir) {
    return MpsseUpdateLowerPins(state, dir, 0xf, /*flush=*/false);
  }

private:
  std::unique_ptr<struct ftdi_context, decltype(&FreeContext)> context_;
  uint8_t buffer_[kBufferSize];
  uint32_t buffer_used_ = 0;

  uint8_t low_pin_state_=0;
  uint8_t low_pin_dir_=0;
};


// =================================== //
// MPSSE data TX clock edge limitation //
// =================================== //
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

// ==================== //
//  I2C Interface Class //
// ==================== //
//
// It should be obvious that it's invalid to interleave the use of the same FtdiDevice.
//
// Pins for I2C
// SCL -> ADBUS0
// SDA -> ADBUS1 and ADBUS2
// Remember to add the pull up resistor if your dongle don't have one.
class MpsseI2c {
public:
  // I2C typically run on 100 or 400 kHz.
  // Postcond after Create: SDA & SCL both hold high. (aka idle state)
  static std::unique_ptr<MpsseI2c> Create(FtdiDevice *dev, float scl_khz = 400);
  virtual ~MpsseI2c();

  // No copy nor move.
  MpsseI2c(const MpsseI2c &) = delete;
  MpsseI2c &operator=(const MpsseI2c &) = delete;
  MpsseI2c &operator=(MpsseI2c &&another) noexcept = delete;
  MpsseI2c(MpsseI2c &&another) noexcept = delete;

  // Convert 7bit addr & rw bit to 8bit data.
  static constexpr uint8_t Addr7ToData(uint8_t addr7, int is_read) {
    return (addr7 << 1) | ((is_read) ? 1 : 0);
  }

  // Precond: SDA & SCL hold high.
  // Postcond: SDA & SCL hold low.
  //
  // SDA ‾‾\____
  // SCL ‾‾‾‾\__
  Status Start();

  // Precond: SDA & SCL hold low.
  // Postcond: SDA & SCL hold low.
  // Do a repeated start.
  //
  // SDA ___/‾‾‾\___
  // SCL _____/‾‾‾\__
  Status Restart();

  // Precond: SDA & SCL hold low.
  // Postcond: SDA & SCL hold high.
  // First bring SCL high, then bring SDA high while SCL is high to signal a stop.
  //
  // SDA ____/‾‾
  // SCL __/‾‾‾‾
  Status Stop();

  // Precond: SDA & SCL hold low.
  // Postcond: SDA & SCL hold low.
  // Clock out 8bits (MSBit first), then immediately read one ack bit.
  //
  // Set *ack to TRUE if ACK is received. Set to NULL if you don't care about ack bit.
  Status WriteByte(uint8_t data, bool *ack);

  // Precond: SDA & SCL hold low.
  // Postcond: SDA & SCL hold low.
  // Clock in n bytes, send an ACK for first n-1 bytes, and send a NACK for last byte.
  // Limited to 320 bytes to avoid overflow the buffer.
  // If you really need to read that many data, call the function multiple times.
  Status ReadBytes(uint16_t len, void* buf);

  // Precond: SDA & SCL hold high.
  // Postcond: SDA & SCL hold high.
  // This is a helper function to issue a I2C transaction in one go.
  // - txlen=0 and rxlen>=0 : Start-IssueRdAddr-ReadBytes-Stop
  // - txlen>0 and rxlen=0  : Start-IssueWrAddr-WriteBytes-Stop
  // - txlen>0 and rxlen>0  : Start-IssueWrAddr-WriteBytes-Restart-IssueRdAddr-ReadBytes-Stop
  // If a len is zero, the corresponding data can be nullptr.
  Status Transaction(uint8_t addr7, const uint8_t* tx_data, int tx_len, void* rx_buf, int rx_len);

private:
  explicit MpsseI2c(FtdiDevice* dev) : dev_(dev) {}
  FtdiDevice* const dev_;
};

// ===================== //
//  WS2812B one-wire LED //
// ===================== //
//
// Only one pin required.
// DATA <- ADBUS1
// The frequency is set to 2.5MHz and use the data pin to simulate the required timing.
// - "0 code" == 0b100 == 0.4us then 0.8us
// - "1 code" == 0b110 == 0.8us then 0.4us
// - "reset"  == at least 125 zeros.
class MpsseWs2812b {
public:
  // It should be obvious that it's invalid to interleave the use of the same FtdiDevice.
  static std::unique_ptr<MpsseWs2812b> Create(FtdiDevice *dev);
  virtual ~MpsseWs2812b();

  // Change color of multiple LEDs. One number for one LED, in that order.
  // Every number represent the RGB value of that LED. Top 8 bits are ignored. Blue in LSB.
  Status SendFrame(std::span<const uint32_t> rgb);
private:
  explicit MpsseWs2812b(FtdiDevice* dev) : dev_(dev) {}

  // Expand one byte to 3 bytes. 0 map to 0b100, 1 map to 0b110. buf is assumed to have size 3.
  static void ExpandByte(uint8_t byte, uint8_t buf[]);
  // Send raw data, all bits must be either "100" or "110". In GRB order. Last bit must be zero.
  Status SendRaw(std::span<const uint8_t> raw);

  FtdiDevice* const dev_;
};

// ===================== //
//  SPI Interface Class  //
// ===================== //
//
// Walkaround for SPI Modes:
//     Mode 0: Use normal two phase CLOCK, WRITE_NEG, READ_POS
//      CLK    __/‾‾\__/‾‾\          __/‾‾\__/‾‾\_
//     MOSI    <===> <===>   MISO    <===> <===>
//
//     Mode 1: Cannot use two phase clock. WRITE_POS is invalid when clock is idle low.
//             Use three phase clock to mimic, we need to read one more bit and discard the first read.
//     (BAD)  CLK    __/‾‾\__/‾‾\_
//     (BAD) MOSI       <===> <===>
//
//      CLK          __/‾‾\_____/‾‾\__       __/‾‾\_____/‾‾\_____/‾‾\__
//     MOSI          <====^=> <====^=>  MISO XX <======> <======> XXXXX
//                                            ^-bogus  ^-bit0   ^-bit1
//
// Pins for SPI
// ADBUS0: CLK. Connect to CLK pin on peripherals.
// ADBUS1: MOSI. Connect to MOSI or SDI (serial data in) pin on peripherals.
// ADBUS2: MISO. Connect to MISO or SDO (serial data out) pin on peripherals.
// ADBUS3: Chip select. TODO: support multiple CS pins.
class MpsseSpi {
public:
  // CPOL: clock polarity: 0-clock idle low, 1-clock idle high
  // CPHA: clock phase   : 0-first half of clock is idle, then inverted.
  //                       1-first half of clock is inverted, then idle.
  // ref: https://www.analog.com/en/resources/analog-dialogue/articles/introduction-to-spi-interface.html
  //
  // clk_mhz: The desired clock frequency in MHz.
  //          Due to FTDI chip restriction, if cpha == 1, the clock duty cycle is not 50/50.
  //          The clock period should >= 3x pause width. (i.e. clk_mhz <= max freq * 2/3)
  static std::unique_ptr<MpsseSpi> Create(FtdiDevice *dev, int cpol, int cpha, float clk_mhz=1);
  virtual ~MpsseSpi();

  // Pull down CS, transmit some bytes then immediately read some bytes, then pull up CS.
  // Can not be both zero length. If a length is zero, the corresponding data can be null.
  //
  // tx_len: bytes to transmit, can be zero.
  // rx_len: bytes to receive, can be zero.
  Status Transaction(const void* tx_data, int tx_len, void* rx_data, int rx_len);

private:
  explicit MpsseSpi(FtdiDevice* dev, int cpol, int cpha) :
    dev_(dev), cpol_(cpol), cpha_(cpha) {}

  FtdiDevice* const dev_;
  const int cpol_;
  const int cpha_;
};

} // namespace mpsse_protocol

#endif // __MPSSE_PROTOCOL_H__
