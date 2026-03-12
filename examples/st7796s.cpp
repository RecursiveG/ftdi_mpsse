// Tested on https://www.lcdwiki.com/4.0inch_Capacitive_SPI_Module_ST7796

#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "mpsse_protocol.h"

#define DIE_IF(cond, fmt, ...)                                                                          \
  do {                                                                                                  \
    if (cond) {                                                                                         \
      fprintf(stderr, fmt "\n", ##__VA_ARGS__);                                                         \
      exit(1);                                                                                          \
    }                                                                                                   \
  } while (0)

#define RETURN_IF_ERR(st)                                                                               \
  do {                                                                                                  \
    Status s = (st);                                                                                    \
    if (!s.ok()) return s;                                                                              \
  } while (0)

#define DIE_IF_ERR(st)                                                                                  \
  do {                                                                                                  \
    Status s = (st);                                                                                    \
    if (!s.ok()) {                                                                                      \
      fprintf(stderr, "%s\n", st.human().c_str());                                                      \
      exit(1);                                                                                          \
    }                                                                                                   \
  } while (0)

using mpsse_protocol::FtdiDevice;
using mpsse_protocol::MpsseGpio;
using mpsse_protocol::MpsseSpi;
using mpsse_protocol::Status;

#define CMD_RDID1 0xda
#define CMD_RDID2 0xdb
#define CMD_RDID3 0xdc
#define CMD_CSCON 0xf0
#define CMD_SPIRC 0xfb

// D/C pin connected to GPIOL0 (ADBUS4)
// nRST pin connected to GPIOL1 (ADBUS5)
class St7796sController {
public:
  St7796sController(MpsseSpi *spi, MpsseGpio *gpio) : spi_(spi), gpio_(gpio) {}

  // Buffer the command to toggle the D/C pin
  Status BufferCommand() { return gpio_->BufferLowerPins(0b0010'0000, 0b0011'0000); }
  Status BufferData() { return gpio_->BufferLowerPins(0b0011'0000, 0b0011'0000); }

  // Issue one byte command then read one byte data
  // Status CommandRead(uint8_t cmd, uint8_t *ret) {
  //   RETURN_IF_ERR(gpio_->BufferLowerPins(0b0010'0000, 0b0011'0000));
  //   return spi_->Transaction(&cmd, 1, &ret, 1);
  // }

  // Issue one byte command then multiple params
  Status CommandWrite(uint8_t cmd, std::initializer_list<uint8_t> params) {
    std::vector<uint8_t> p{params.begin(), params.end()};

    RETURN_IF_ERR(BufferCommand());
    RETURN_IF_ERR(spi_->Transaction(&cmd, 1, nullptr, 0));
    if (p.size() > 0) {
      RETURN_IF_ERR(BufferData());
      RETURN_IF_ERR(spi_->Transaction(p.data(), p.size(), nullptr, 0));
    }
    return Status::Ok();
  }

  // Drive nRST pin Low then high.
  Status Reset() {
    gpio_->SetLowerPins(0b0000'0000, 0b0011'0000);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    gpio_->SetLowerPins(0b0010'0000, 0b0011'0000);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return Status::Ok();
  }

  Status Init() {
    // clang-format off
    // 0xf0 control was here
    RETURN_IF_ERR(CommandWrite(0x36, {0x48}));       // MADCTL = MX | RGB (BGR?)
    RETURN_IF_ERR(CommandWrite(0x3a, {0x05}));       // Intf Pixel Fmt = D2 | D0 (16bit)
    RETURN_IF_ERR(CommandWrite(0xb0, {0x80}));       // IFMODE = SPI_EN !! Why is this bit set?
    RETURN_IF_ERR(CommandWrite(0xb6, {0x00, 0x02})); // !! datasheet say has 3 param, why only 2 here?
    RETURN_IF_ERR(CommandWrite(0xb5, {0x02, 0x03, 0x00, 0x04})); // Blank porch control
    RETURN_IF_ERR(CommandWrite(0xb1, {0x80, 0x10}));             // Frame rate control
    RETURN_IF_ERR(CommandWrite(0xb4, {0x00}));                   // Display inversion control
    RETURN_IF_ERR(CommandWrite(0xb7, {0xc6}));                   // Entry mode set
    RETURN_IF_ERR(CommandWrite(0xc5, {0x1c}));                   // Vcom ctrl - some voltage stuff?
    RETURN_IF_ERR(CommandWrite(0xe4, {0x31}));                   // !! undocumented cmd
    RETURN_IF_ERR(CommandWrite(0xe8, {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33}));
                                                                 // Display output ctrl adjust
    RETURN_IF_ERR(CommandWrite(0xc2, {})); // Power ctrl 3 !! where's the param?
    RETURN_IF_ERR(CommandWrite(0xa7, {})); // !! undocumented cmd
    RETURN_IF_ERR(CommandWrite(0xe0, {0xF0, 0x09, 0x13, 0x12, 0x12, 0x2B, 0x3C, 0x44, 0x4B, 0x1B, 0x18, 0x17, 0x1D, 0x21}));
                                           // positive gamma ctrl
    RETURN_IF_ERR(CommandWrite(0xe1, {0xF0, 0x09, 0x13, 0x0C, 0x0D, 0x27, 0x3B, 0x44, 0x4D, 0x0B, 0x17, 0x17, 0x1D, 0x21}));
                                           // negative gamma ctrl
    // 0xf0 control was here
    RETURN_IF_ERR(CommandWrite(0x13, {})); // normal display mode on
    RETURN_IF_ERR(CommandWrite(0x11, {})); // sleep out
    RETURN_IF_ERR(CommandWrite(0x29, {})); // display on
    // clang-format on
    return Status::Ok();
  }

  Status ReadId(uint16_t *id) {
    uint32_t tmp;
    uint8_t cmd = 0xd3;
    RETURN_IF_ERR(BufferCommand());
    RETURN_IF_ERR(spi_->Transaction(&cmd, 1, &tmp, 4));
    // This command has 9bits of dummy data before emitting data.
    // Also swap the byte order.
    tmp = ntohl(tmp);
    tmp >>= 7; // 32 - 9 - 16 = 7
    *id = tmp;
    return Status::Ok();
  }

  Status FillColor(uint16_t color565) {
    constexpr int kTotalBytes = 320 * 480 * 2;
    int sent = 0;
    auto buf = std::make_unique<uint8_t[]>(kTotalBytes);
    for (int i = 0; i < 320 * 480; i++) {
      buf[i * 2] = color565 >> 8;
      buf[i * 2 + 1] = color565;
    }

    RETURN_IF_ERR(CommandWrite(0x2a, {0, 0, 1, 0x3f})); // column = [0, 320)
    RETURN_IF_ERR(CommandWrite(0x2b, {0, 0, 1, 0xdf})); // column = [0, 480)
    RETURN_IF_ERR(CommandWrite(0x2c, {}));
    RETURN_IF_ERR(BufferData());
    while (sent < kTotalBytes) {
      int to_send = kTotalBytes - sent;
      if (to_send > 30720) to_send = 30720;
      RETURN_IF_ERR(spi_->Transaction(buf.get() + sent, to_send, nullptr, 0));
      sent += to_send;
    }
    return Status::Ok();
  }

  Status FillPic(const std::vector<uint8_t> data) {
    constexpr int kTotalBytes = 320 * 480 * 2;
    int sent = 0;
    DIE_IF(data.size() != kTotalBytes, "invalid data size");

    RETURN_IF_ERR(CommandWrite(0x2a, {0, 0, 1, 0x3f})); // column = [0, 320)
    RETURN_IF_ERR(CommandWrite(0x2b, {0, 0, 1, 0xdf})); // column = [0, 480)
    RETURN_IF_ERR(CommandWrite(0x2c, {}));
    RETURN_IF_ERR(BufferData());
    while (sent < kTotalBytes) {
      int to_send = kTotalBytes - sent;
      if (to_send > 30720) to_send = 30720;
      RETURN_IF_ERR(spi_->Transaction(data.data() + sent, to_send, nullptr, 0));
      sent += to_send;
    }
    return Status::Ok();
  }

private:
  MpsseSpi *spi_;
  MpsseGpio *gpio_;
};

// AI-generated code.
std::vector<uint8_t> LoadPicture(std::string path) {
  cv::Mat src = cv::imread(path);
  DIE_IF(src.empty(), "cannot open image");

  if (src.cols > src.rows) {
    cv::rotate(src, src, cv::ROTATE_90_COUNTERCLOCKWISE);
  }

  // Calculate aspect ratio and scaling
  double scale = std::min((double)320 / src.cols, (double)480 / src.rows);
  int new_w = static_cast<int>(src.cols * scale);
  int new_h = static_cast<int>(src.rows * scale);

  // Resize the image to fit the scale
  cv::Mat resized;
  cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, 3);

  // Create a black canvas (480x320) and center the resized image
  cv::Mat canvas = cv::Mat::zeros(cv::Size(320, 480), CV_8UC3);

  int x_offset = (320 - new_w) / 2;
  int y_offset = (480 - new_h) / 2;

  // Copy resized image into the center of the black canvas
  resized.copyTo(canvas(cv::Rect(x_offset, y_offset, new_w, new_h)));

  // 2. Create a vector (array) to store the 16-bit RGB565 values
  std::vector<uint8_t> rgb565_array;
  rgb565_array.reserve(320 * 480 * 2);

  // 3. Process pixels
  for (int r = 0; r < 480; ++r) {
    for (int c = 0; c < 320; ++c) {
      // Get the BGR pixel
      cv::Vec3b pixel = canvas.at<cv::Vec3b>(r, c);
      uint8_t b = pixel[0];
      uint8_t g = pixel[1];
      uint8_t r_val = pixel[2];

      // 4. Convert to RGB565
      uint16_t rgb565 = ((r_val & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);

      rgb565_array.push_back(rgb565 >> 8);
      rgb565_array.push_back(rgb565);
    }
  }

  return rgb565_array;
}

int main(int argc, char *argv[]) {
  std::unique_ptr<FtdiDevice> dev = FtdiDevice::OpenVendorProduct(0x0403, 0x6010, INTERFACE_A);
  DIE_IF(dev == nullptr, "Cannot open dev");
  // Id read command gives wrong data at 15MHz
  // The next available is 10MHz
  // Max: 10 * 1e6 / (320*480*2*8) = 4fps
  std::unique_ptr<MpsseSpi> spi = MpsseSpi::Create(dev.get(), 0, 0, 10);
  DIE_IF(spi == nullptr, "Cannot open SPI");
  MpsseGpio gpio = spi->Gpio();

  St7796sController lcd{spi.get(), &gpio};
  DIE_IF_ERR(lcd.Reset());
  DIE_IF_ERR(lcd.Init());

  {
    uint16_t id;
    DIE_IF_ERR(lcd.ReadId(&id));
    std::printf("LCD ID: %#x\n", id);
  }

  if (argc >= 2) {
    auto pic_data = LoadPicture(argv[1]);
    lcd.FillPic(pic_data);
    while (true) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } else {
    uint16_t colors[] = {0xf800, 0x07e0, 0x001f, 0x0000, 0xffff};
    int index = 0;
    while (true) {
      lcd.FillColor(colors[(index++) % 5]);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  return 0;
}
