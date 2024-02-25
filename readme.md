This is a CC1101 transciever component that works with esphome's remote_transmitter/remote_receiver.
  
It can be compiled with Arduino and esp-idf framework and should support any esphome compatible board through the SPI Bus.

The source code is a mashup of the following github projects with some special esphome sauce:

- https://github.com/dbuezas/esphome-cc1101 (the original esphome component)
- https://github.com/nistvan86/esphome-q7rf (how to use esphome with spi)
- https://github.com/LSatan/SmartRC-CC1101-Driver-Lib (cc1101 setup code)

TODO:

- Convert it from Switch to a Sensor and return some diagnostic values (rssi...)
- RP2040? (USE_RP2040)
- Libretiny? (USE_LIBRETINY)