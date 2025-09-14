## Control devices using FTDI MPSSE

Controls peripherals using MPSSE, which is inside many FTDI chips. Example FT232H or FT2232H. Note: the FT232R does NOT support MPSSE.
This repo uses [libftdi1](https://www.intra2net.com/en/developer/libftdi/index.php) and tested on an FT2232H device.

Implemented protocols:
- I<sup>2</sup>C
  - Example: MCP9808 temperature sensor.
- WS2812B LED
  - Tested on a light strip with 144 LEDs.

Note about USB device address:
- Every USB device is on some bus, and is assigned a unique device number on that bus.
  e.g. `9-13` means bus 9 device 13.
- Linux use a similar notation for the ports `bus-port[.port][.port]` which can be confusing.
  e.g. `9-3` means port 3 on the root hub of bus 9. If there's another hub, append the port number
  with a dot. e.g. `9-3.1` means the device on port 1 of the hub that is on port 3 of the root.
- Sometimes the configuration and interface are appended: `bus-port.port:config.interface`. e.g.
  `9-3.1:1.0`. Configuration and interface are standard concepts defined by the USB spec.
