This version is no longer maintained, please test directly the [esphome pull request](https://github.com/esphome/esphome/pull/6300).

```
external_components:
  - source: github://pr#6300
    components: [ cc1101 ]
```

[Documentation preview](https://deploy-preview-3897--esphome.netlify.app/components/cc1101.html)

---

This is a CC1101 transceiver component that works with esphome's remote_transmitter/remote_receiver.
  
It can be compiled with Arduino or the esp-idf framework and should support any esphome compatible board through the SPI Bus.

On ESP8266, you can use the same pin for GDO and GD2 (it is an optional parameter).

The source code is a mashup of the following github projects with some special esphome sauce:

- https://github.com/dbuezas/esphome-cc1101 (the original esphome component)
- https://github.com/nistvan86/esphome-q7rf (how to use esphome with spi)
- https://github.com/LSatan/SmartRC-CC1101-Driver-Lib (cc1101 setup code)

TODO:

- RP2040? (USE_RP2040)
- Libretiny? (USE_LIBRETINY)

Tested with:

| Board         | MISO     | MOSI    | SCK     | CSN     | GDO0    | GDO2    | SDA     | SCL     |      |
| :------------ | :------: | :-----: | :-----: | :-----: | :-----: | :-----: | :-----: | :-----: | :------- |
| nodemcu-32s   | 19 | 23 | 18 | 5  | 32 | 33 |   |   |  |
| lolin_s2_mini | 37 | 35 | 36 | 34 | 8  | 9  |   |   |  |
| c3 supermini  | 5  | 7  | 6  | 10 | 3  | 4  | 0 | 1 | add one dummy transmitter [#2934](https://github.com/esphome/issues/issues/2934) |
| nodemcuv2     | 12 | 13 | 14 | 15 | 5 |  | 4 | TX | still possible to use i2c with TX |
| d1_mini       | 12 | 13 | 14 | 15 | 5 |  | 4 | TX |  |

SDA/SCL is not needed of course, they are just there as suggestions in case you also need I2C on such a low pin count board.
