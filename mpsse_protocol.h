#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <ftdi.h>

namespace mpsse_protocol {

class FtdiDevice {
public:
  static std::unique_ptr<FtdiDevice>
  OpenVendorProduct(uint16_t id_vendor, uint16_t id_product,
                    enum ftdi_interface intf = INTERFACE_ANY);
  // This is the device number. Not to be confused by the port number which may also shown
  // as "x-y"
  static std::unique_ptr<FtdiDevice>
  OpenBusDevice(int bus, int device, enum ftdi_interface intf = INTERFACE_ANY);

  // No copy nor move.
  FtdiDevice(const FtdiDevice &) = delete;
  FtdiDevice &operator=(const FtdiDevice &) = delete;
  FtdiDevice &operator=(FtdiDevice &&another) noexcept = delete;
  FtdiDevice(FtdiDevice &&another) noexcept = delete;

  // The context will be freed on destruction.
  explicit FtdiDevice(struct ftdi_context *context) : context_(context) {}
  virtual ~FtdiDevice();
  struct ftdi_context *context() { return context_; }

  //
  // Helper functions used by different interface classes.
  // All these functions return 0 on success unless otherwise stated.
  //

  // First stash bytes into a buffer, then use BufferFlush() to flush all bytes to the device.
  // If there are not enough space in the buffer, it will return a none zero value, and the buffer
  // is unchanged.
  static constexpr int kBufferSize = 512;
  void BufferClear();
  int BufferByte(uint8_t data);
  int BufferBytes(std::span<uint8_t> data);
  int BufferFlush();

  // Wrapper around ftdi_read_data()
  // Return -1 if error, 0 if success
  int Read(void* buf, int32_t len);

  // Synchronize the MPSSE state, must be run before other MPSSE commands.
  // Note the implementation isn't identical to AN_135
  int MpsseSync();
  int MpsseSetClockFreq(float khz, bool three_phase, bool adaptive);

private:
  struct ftdi_context *const context_;
  uint8_t buffer_[kBufferSize];
  uint32_t buffer_len_ = 0;
};

// Pins for I2C
// SCL -> ADBUS0
// SDA -> ADBUS1 and ADBUS2
// Remember to add the pull up resistor if your dongle don't have one.
class MpsseI2c {
public:
  // It should be obvious that it's invalid to interleave the use of the same FtdiDevice.
  // I2C typically run on 100 or 400 kHz.
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
  // Return 0 if success, -1 if any error occurs
  //
  // SDA ‾‾\____
  // SCL ‾‾‾‾\__
  int Start();

  // Precond: SDA & SCL hold low.
  // Postcond: SDA & SCL hold low.
  // Return 0 if success, -1 if any error occurs
  // Do a repeated start.
  //
  // SDA ___/‾‾‾\___
  // SCL _____/‾‾‾\__
  int Restart();

  // Precond: SDA & SCL hold low.
  // Postcond: SDA & SCL hold high.
  // Return 0 if success, -1 if any error occurs
  // First bring SCL high, then bring SDA high while SCL is high to signal a stop.
  //
  // SDA ____/‾‾
  // SCL __/‾‾‾‾
  int Stop();

  // Precond: SDA & SCL hold low.
  // Postcond: SDA & SCL hold low.
  // Clock out 8bits (MSBit first), then immediately read one ack bit.
  // Returns 1 if ACK, or 0 if NACK. -1 if error occurs.
  int WriteByte(uint8_t data);

  // Precond: SDA & SCL hold low.
  // Postcond: SDA & SCL hold low.
  // Clock in n bytes, send an ACK for first n-1 bytes, and send a NACK for last byte.
  // Return 0 if ok, -1 if failure.
  int ReadBytes(uint16_t len, void* buf);

  // Precond: SDA & SCL hold high.
  // Postcond: SDA & SCL hold high.
  // This is a helper function to issue a I2C transaction in one go.
  // - txlen=0 and rxlen>=0 : Start-IssueRdAddr-ReadBytes-Stop
  // - txlen>0 and rxlen=0  : Start-IssueWrAddr-WriteBytes-Stop
  // - txlen>0 and rxlen>0  : Start-IssueWrAddr-WriteBytes-Restart-IssueRdAddr-ReadBytes-Stop
  // If a len is zero, the corresponding data can be nullptr.
  //
  // Return 0 if ok, -1 if failure, -2 if get NACK when ACK is expected.
  int Transaction(uint8_t addr7, const uint8_t* tx_data, int tx_len, void* rx_buf, int rx_len);

private:
  explicit MpsseI2c(FtdiDevice* dev) : dev_(dev) {}

  // For comments on private functions, see cpp file.
  int InitializeI2cPins();
  int SetLowerPins(uint8_t state, uint8_t dir);

  FtdiDevice* const dev_;
};

} // namespace mpsse_protocol
