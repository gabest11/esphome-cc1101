#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace cc1101 {

class CC1101Switch 
  : public switch_::Switch,
    public PollingComponent,
    public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1KHZ>
{
protected:
  int32_t _gdo0;
  int32_t _gdo2;
  uint32_t _bandwidth;
  uint32_t _frequency;

  uint8_t _partnum;
  uint8_t _version;
  int32_t _last_rssi;

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

  bool _mode;
  uint8_t _modulation;
  uint8_t _frend0;
  uint8_t _chan;
  int8_t _pa;
  uint8_t _last_pa;
  uint8_t _m4RxBw;
  uint8_t _m4DaRa;
  uint8_t _m2DCOFF;
  uint8_t _m2MODFM;
  uint8_t _m2MANCH;
  uint8_t _m2SYNCM;
  uint8_t _m1FEC;
  uint8_t _m1PRE;
  uint8_t _m1CHSP;
  uint8_t _trxstate;
  uint8_t _clb[4][2];

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
  CC1101Switch();

  void set_cc1101_gdo0(uint8_t pin) { _gdo0 = pin; if(_gdo2 == -1) _gdo2 = pin; }
  void set_cc1101_gdo2(uint8_t pin) { _gdo2 = pin; }
  void set_cc1101_bandwidth(uint32_t bandwidth) { _bandwidth = bandwidth; }
  void set_cc1101_frequency(uint32_t frequency) { _frequency = frequency; }

  void setup() override;
  void update() override;
  void write_state(bool state) override {} // TODO
  void dump_config() override;

  int32_t get_rssi();
  uint8_t get_lqi();

  void begin_tx();
  void end_tx();
};

template<typename... Ts> class BeginTxAction : public Action<Ts...>, public Parented<CC1101Switch>
{
public:
  void play(Ts... x) override { this->parent_->begin_tx(); }
};

template<typename... Ts> class EndTxAction : public Action<Ts...>, public Parented<CC1101Switch>
{
public:
  void play(Ts... x) override { this->parent_->end_tx(); }
};

} // namespace cc1101
} // namespace esphome

#define get_cc1101(id) (*((esphome::cc1101::CC1101Switch*)id))
