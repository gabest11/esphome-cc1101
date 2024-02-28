#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace cc1101 {

class CC1101 
  : public sensor::Sensor,
    public PollingComponent,
    public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1KHZ>
{
protected:
  InternalGPIOPin* gdo0_;
  InternalGPIOPin* gdo2_;
  uint32_t bandwidth_;
  uint32_t frequency_;
  sensor::Sensor* rssi_sensor_;
  sensor::Sensor* lqi_sensor_;

  uint8_t partnum_;
  uint8_t version_;
  int32_t last_rssi_;
  int32_t last_lqi_;

  bool reset();
  void send_cmd(uint8_t cmd);
  uint8_t read_register(uint8_t reg);
  uint8_t read_config_register(uint8_t reg);
  uint8_t read_status_register(uint8_t reg);
  void read_register_burst(uint8_t reg, uint8_t* buffer, size_t length);
  void write_register(uint8_t reg, uint8_t* value, size_t length);
  void write_register(uint8_t reg, uint8_t value);
  void write_register_burst(uint8_t reg, uint8_t* buffer, size_t length);
  //bool send_data(const uint8_t* data, size_t length);

  // ELECHOUSE_CC1101 stuff

  bool mode_;
  uint8_t modulation_;
  uint8_t frend0_;
  uint8_t chan_;
  int8_t pa_;
  uint8_t last_pa_;
  uint8_t m4RxBw_;
  uint8_t m4DaRa_;
  uint8_t m2DCOFF_;
  uint8_t m2MODFM_;
  uint8_t m2MANCH_;
  uint8_t m2SYNCM_;
  uint8_t m1FEC_;
  uint8_t m1PRE_;
  uint8_t m1CHSP_;
  uint8_t trxstate_;
  uint8_t clb_[4][2];

  void set_mode(bool s);
  void set_frequency(uint32_t f);
  void set_modulation(uint8_t m);
  void set_pa(int8_t pa);
  void set_clb(uint8_t b, uint8_t s, uint8_t e);
  void set_rxbw(uint32_t bw);
  void set_tx();
  void set_rx();
  void set_sres();
  void set_sidle();
  void set_sleep();

  void split_MDMCFG2();
  void split_MDMCFG4();

public:
  CC1101();

  void set_config_gdo0(InternalGPIOPin* pin);
  void set_config_gdo2(InternalGPIOPin* pin);
  void set_config_bandwidth(uint32_t bandwidth);
  void set_config_frequency(uint32_t frequency);
  void set_config_rssi_sensor(sensor::Sensor* rssi_sensor);
  void set_config_lqi_sensor(sensor::Sensor* lqi_sensor);

  void setup() override;
  void update() override;
  void dump_config() override;

  int32_t get_rssi();
  uint8_t get_lqi();

  void begin_tx();
  void end_tx();
};

template<typename... Ts> class BeginTxAction : public Action<Ts...>, public Parented<CC1101>
{
public:
  void play(Ts... x) override { this->parent_->begin_tx(); }
};

template<typename... Ts> class EndTxAction : public Action<Ts...>, public Parented<CC1101>
{
public:
  void play(Ts... x) override { this->parent_->end_tx(); }
};

} // namespace cc1101
} // namespace esphome

