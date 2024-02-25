/*
  https://github.com/gabest11/esphome-cc1101

  This is a CC1101 transceiver component that works with esphome's remote_transmitter/remote_receiver.
  
  It can be compiled with Arduino and esp-idf framework and should support any esphome compatible board through the SPI Bus.

  The source code is a mashup of the following github projects with some special esphome sauce:

  https://github.com/dbuezas/esphome-cc1101 (the original esphome component)
  https://github.com/nistvan86/esphome-q7rf (how to use esphome with spi)
  https://github.com/LSatan/SmartRC-CC1101-Driver-Lib (cc1101 setup code)

  TODO: Convert it from Switch to a Sensor and return some diagnostic values (rssi...)
  TODO: RP2040? (USE_RP2040)
  TODO: Libretiny? (USE_LIBRETINY)
*/

#include "esphome/core/log.h"
#include "cc1101.h"
#include "cc1101defs.h"

#ifdef USE_ARDUINO
#include <Arduino.h>
#else // USE_ESP_IDF
#include <driver/gpio.h>
long map(long x, long in_min, long in_max, long out_min, long out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
#endif

namespace esphome {
namespace cc1101 {

static const char *TAG = "cc1101.switch";

uint8_t PA_TABLE[8]     {0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00};
//                       -30  -20  -15  -10   0    5    7    10
uint8_t PA_TABLE_315[8] {0x12,0x0D,0x1C,0x34,0x51,0x85,0xCB,0xC2};             // 300 - 348
uint8_t PA_TABLE_433[8] {0x12,0x0E,0x1D,0x34,0x60,0x84,0xC8,0xC0};             // 387 - 464
//                        -30  -20  -15  -10  -6    0    5    7    10   12
uint8_t PA_TABLE_868[10] {0x03,0x17,0x1D,0x26,0x37,0x50,0x86,0xCD,0xC5,0xC0};  // 779 - 899.99
//                        -30  -20  -15  -10  -6    0    5    7    10   11
uint8_t PA_TABLE_915[10] {0x03,0x0E,0x1E,0x27,0x38,0x8E,0x84,0xCC,0xC3,0xC0};  // 900 - 928

CC1101Switch::CC1101Switch() : PollingComponent(1000)
{
  _gdo0 = -1;
  _gdo2 = -1;
  _bandwidth = 200;
  _frequency = 433920;

  _mode = false;
  _modulation = 2;
  _chan = 0;
  _pa = 12;
  _last_pa = -1;
  _m4RxBw = 0;
  _trxstate = 0;
  _clb[0][0] = 24; _clb[0][1] = 28;
  _clb[1][0] = 31; _clb[1][1] = 38;
  _clb[2][0] = 65; _clb[2][1] = 76;
  _clb[3][0] = 77; _clb[3][1] = 79;
}

void CC1101Switch::setup()
{
#ifdef USE_ARDUINO
  pinMode(_gdo0, OUTPUT);
  pinMode(_gdo2, INPUT);
#else  
  //gpio_reset_pin((gpio_num_t)_gdo0);
  //gpio_reset_pin((gpio_num_t)_gdo2);
  gpio_set_direction((gpio_num_t)_gdo0, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)_gdo2, GPIO_MODE_INPUT);
#endif

  this->spi_setup();

  if(!reset())
  {
    ESP_LOGE(TAG, "Failed to reset CC1101 modem. Check connection.");
    return;
  }

  // ELECHOUSE_cc1101.Init();

  write_register(CC1101_FSCTRL1, 0x06);

  set_mode(false);
  set_frequency(_frequency);

  write_register(CC1101_MDMCFG1, 0x02);
  write_register(CC1101_MDMCFG0, 0xF8);
  write_register(CC1101_CHANNR, _chan);
  write_register(CC1101_DEVIATN, 0x47);
  write_register(CC1101_FREND1, 0x56);
  write_register(CC1101_MCSM0, 0x18);
  write_register(CC1101_FOCCFG, 0x16);
  write_register(CC1101_BSCFG, 0x1C);
  write_register(CC1101_AGCCTRL2, 0xC7);
  write_register(CC1101_AGCCTRL1, 0x00);
  write_register(CC1101_AGCCTRL0, 0xB2);
  write_register(CC1101_FSCAL3, 0xE9);
  write_register(CC1101_FSCAL2, 0x2A);
  write_register(CC1101_FSCAL1, 0x00);
  write_register(CC1101_FSCAL0, 0x1F);
  write_register(CC1101_FSTEST, 0x59);
  write_register(CC1101_TEST2, 0x81);
  write_register(CC1101_TEST1, 0x35);
  write_register(CC1101_TEST0, 0x09);
  write_register(CC1101_PKTCTRL1, 0x04);
  write_register(CC1101_ADDR, 0x00);
  write_register(CC1101_PKTLEN, 0x00);

  // ELECHOUSE_cc1101.setRxBW(_bandwidth);

  set_rxbw(_bandwidth);

  // ELECHOUSE_cc1101.setMHZ(_freq);

  set_frequency(_frequency); // TODO: already set

  //

  set_rx();

  //

  ESP_LOGI(TAG, "CC1101 initialized.");
}

void CC1101Switch::update()
{
}

void CC1101Switch::dump_config()
{
//  ESP_LOGCONFIG(TAG, "CC1101:");
#ifdef USE_ARDUINO
  ESP_LOGCONFIG(TAG, "CC1101 (Arduino):");
#else // USE_ESP_IDF
  ESP_LOGCONFIG(TAG, "CC1101 (esp-idf):");
#endif
  LOG_PIN("  CC1101 CS Pin: ", this->cs_);
  ESP_LOGCONFIG(TAG, "  CC1101 GDO0: %d", _gdo0);
  ESP_LOGCONFIG(TAG, "  CC1101 GDO2: %d", _gdo2);
  ESP_LOGCONFIG(TAG, "  CC1101 Bandwith: %d KHz", _bandwidth);
  ESP_LOGCONFIG(TAG, "  CC1101 Frequency: %d KHz", _frequency);
}

bool CC1101Switch::reset()
{
  // Chip reset sequence. CS wiggle (CC1101 manual page 45)

  //this->disable(); // esp-idf calls end_transaction and asserts, because no begin_transaction was called
  this->cs_->digital_write(false);
  delayMicroseconds(5);
  //this->enable();
  this->cs_->digital_write(true);
  delayMicroseconds(10);
  //this->disable();
  this->cs_->digital_write(false);
  delayMicroseconds(41);
  
  send_cmd(CC1101_SRES);

  ESP_LOGD(TAG, "Issued CC1101 reset sequence.");

  // Read part number and version

  uint8_t partnum = read_status_register(CC1101_PARTNUM);
  uint8_t version = read_status_register(CC1101_VERSION);

  ESP_LOGI(TAG, "CC1101 found with partnum: %02x and version: %02x", partnum, version);

  return version > 0;
}

void CC1101Switch::send_cmd(uint8_t cmd)
{
  this->enable();
  this->transfer_byte(cmd);
  this->disable();
}

uint8_t CC1101Switch::read_register(uint8_t reg)
{
  this->enable();
  this->transfer_byte(reg);
  uint8_t value = this->transfer_byte(0);
  this->disable();
  return value;
}

uint8_t CC1101Switch::read_config_register(uint8_t reg)
{
  return read_register(reg | CC1101_READ_SINGLE);
}

uint8_t CC1101Switch::read_status_register(uint8_t reg)
{
  return read_register(reg | CC1101_READ_BURST);
}

void CC1101Switch::read_register_burst(uint8_t reg, uint8_t* buffer, size_t length)
{
  this->enable();
  this->write_byte(reg | CC1101_READ_BURST);
  this->read_array(buffer, length);
  this->disable();
}
void CC1101Switch::write_register(uint8_t reg, uint8_t* value, size_t length)
{
  this->enable();
  this->transfer_byte(reg);
  this->transfer_array(value, length);
  this->disable();
}

void CC1101Switch::write_register(uint8_t reg, uint8_t value)
{
  uint8_t arr[1] = {value};
  write_register(reg, arr, 1);
}

void CC1101Switch::write_register_burst(uint8_t reg, uint8_t* value, size_t length)
{
  write_register(reg | CC1101_WRITE_BURST, value, length);
}
/*
bool CC1101Switch::send_data(const uint8_t* data, size_t length)
{
  uint8_t buffer[length];
  
  memcpy(buffer, data, lenght);

  send_cmd(CC1101_SIDLE);
  send_cmd(CC1101_SFRX);
  send_cmd(CC1101_SFTX);

  write_register_burst(CC1101_TXFIFO, buffer, length);

  send_cmd(CC1101_STX);

  uint8_t state = read_status_register(CC1101_MARCSTATE) & 0x1f;

  if(state != CC1101_MARCSTATE_TX && state != CC1101_MARCSTATE_TX_END && state != CC1101_MARCSTATE_RXTX_SWITCH)
  {
    ESP_LOGE(TAG, "CC1101 in invalid state after sending, returning to idle. State: 0x%02x", state);
    send_cmd(CC1101_SIDLE);
    return false;
  }

  return true;
}
*/

// ELECHOUSE_CC1101 stuff

void CC1101Switch::set_mode(bool s)
{
  _mode = s;

  if(s)
  {
    write_register(CC1101_IOCFG2, 0x0B);
    write_register(CC1101_IOCFG0, 0x06);
    write_register(CC1101_PKTCTRL0, 0x05);
    write_register(CC1101_MDMCFG3, 0xF8);
    write_register(CC1101_MDMCFG4, 11 + _m4RxBw);
  }
  else
  {
    write_register(CC1101_IOCFG2, 0x0D);
    write_register(CC1101_IOCFG0, 0x0D);
    write_register(CC1101_PKTCTRL0, 0x32);
    write_register(CC1101_MDMCFG3, 0x93);
    write_register(CC1101_MDMCFG4, 7 + _m4RxBw);
  }
  
  set_modulation(_modulation);
}

void CC1101Switch::set_modulation(uint8_t m)
{
  if(m > 4) m = 4;

  _modulation = m;

  split_MDMCFG2();

  switch(m)
  {
  case 0: _m2MODFM = 0x00; _frend0 = 0x10; break; // 2-FSK
  case 1: _m2MODFM = 0x10; _frend0 = 0x10; break; // GFSK
  case 2: _m2MODFM = 0x30; _frend0 = 0x11; break; // ASK
  case 3: _m2MODFM = 0x40; _frend0 = 0x10; break; // 4-FSK
  case 4: _m2MODFM = 0x70; _frend0 = 0x10; break; // MSK
  }

  write_register(CC1101_MDMCFG2, _m2DCOFF + _m2MODFM + _m2MANCH + _m2SYNCM);
  write_register(CC1101_FREND0, _frend0);

  set_pa(_pa);
}

void CC1101Switch::set_pa(int8_t pa)
{
  _pa = pa;

  int a;

  if(_frequency >= 300000 && _frequency <= 348000)
  {
    if(pa <= -30) a = PA_TABLE_315[0];
    else if(pa > -30 && pa <= -20) a = PA_TABLE_315[1];
    else if(pa > -20 && pa <= -15) a = PA_TABLE_315[2];
    else if(pa > -15 && pa <= -10) a = PA_TABLE_315[3];
    else if(pa > -10 && pa <= 0) a = PA_TABLE_315[4];
    else if(pa > 0 && pa <= 5) a = PA_TABLE_315[5];
    else if(pa > 5 && pa <= 7) a = PA_TABLE_315[6];
    else a = PA_TABLE_315[7];
    _last_pa = 1;
  }
  else if(_frequency >= 378000 && _frequency <= 464000)
  {
    if(pa <= -30) a = PA_TABLE_433[0];
    else if(pa > -30 && pa <= -20) a = PA_TABLE_433[1];
    else if(pa > -20 && pa <= -15) a = PA_TABLE_433[2];
    else if(pa > -15 && pa <= -10) a = PA_TABLE_433[3];
    else if(pa > -10 && pa <= 0) a = PA_TABLE_433[4];
    else if(pa > 0 && pa <= 5) a = PA_TABLE_433[5];
    else if(pa > 5 && pa <= 7) a = PA_TABLE_433[6];
    else a = PA_TABLE_433[7];
    _last_pa = 2;
  }
  else if(_frequency >= 779000 && _frequency < 900000)
  {
    if(pa <= -30) a = PA_TABLE_868[0];
    else if(pa > -30 && pa <= -20) a = PA_TABLE_868[1];
    else if(pa > -20 && pa <= -15) a = PA_TABLE_868[2];
    else if(pa > -15 && pa <= -10) a = PA_TABLE_868[3];
    else if(pa > -10 && pa <= -6) a = PA_TABLE_868[4];
    else if(pa > -6 && pa <= 0) a = PA_TABLE_868[5];
    else if(pa > 0 && pa <= 5) a = PA_TABLE_868[6];
    else if(pa > 5 && pa <= 7) a = PA_TABLE_868[7];
    else if(pa > 7 && pa <= 10) a = PA_TABLE_868[8];
    else a = PA_TABLE_868[9];
    _last_pa = 3;
  }
  else if(_frequency >= 900000 && _frequency <= 928000)
  {
    if(pa <= -30) a = PA_TABLE_915[0];
    else if(pa > -30 && pa <= -20) a = PA_TABLE_915[1];
    else if(pa > -20 && pa <= -15) a = PA_TABLE_915[2];
    else if(pa > -15 && pa <= -10) a = PA_TABLE_915[3];
    else if(pa > -10 && pa <= -6) a = PA_TABLE_915[4];
    else if(pa > -6 && pa <= 0) a = PA_TABLE_915[5];
    else if(pa > 0 && pa <= 5) a = PA_TABLE_915[6];
    else if(pa > 5 && pa <= 7) a = PA_TABLE_915[7];
    else if(pa > 7 && pa <= 10) a = PA_TABLE_915[8];
    else a = PA_TABLE_915[9];
    _last_pa = 4;
  }
  else
  {
    ESP_LOGE(TAG, "CC1101 set_pa(%d) frequency out of range: %d", pa, _frequency);
    return;
  }

  if(_modulation == 2)
  {
    PA_TABLE[0] = 0;
    PA_TABLE[1] = a;
  }
  else
  {
    PA_TABLE[0] = a;
    PA_TABLE[1] = 0;
  }

  write_register_burst(CC1101_PATABLE, PA_TABLE, sizeof(PA_TABLE));
}

void CC1101Switch::set_frequency(uint32_t f)
{
  _frequency = f;

  uint8_t freq2 = 0;
  uint8_t freq1 = 0;
  uint8_t freq0 = 0;

  float mhz = (float)f / 1000;

  while(true)
  {
    if(mhz >= 26) { mhz -= 26; freq2++; }
    else if(mhz >= 0.1015625) { mhz -= 0.1015625; freq1++; }
    else if(mhz >= 0.00039675) { mhz -= 0.00039675; freq0++; }
    else break;
  }

  /*
  // TODO: impossible, freq0 being uint8_t, also 0.1015625/0.00039675 = 255.9861373660996, it would never reach 256
  if(freq0 > 255)
  {
    freq1 += 1;
    freq0 -= 256;
  }
  */

  write_register(CC1101_FREQ2, freq2);
  write_register(CC1101_FREQ1, freq1);
  write_register(CC1101_FREQ0, freq0);

  // calibrate

  mhz = (float)f / 1000;

  if(mhz >= 300 && mhz <= 348)
  {
    write_register(CC1101_FSCTRL0, map(mhz, 300, 348, _clb[0][0], _clb[0][1]));

    if(mhz < 322.88)
    {
      write_register(CC1101_TEST0, 0x0B);
    }
    else
    {
      write_register(CC1101_TEST0, 0x09);

      uint8_t s = read_status_register(CC1101_FSCAL2);

      if(s < 32)
      {
        write_register(CC1101_FSCAL2, s + 32);
      }

      if(_last_pa != 1) set_pa(_pa);
    }
  }
  else if(mhz >= 378 && mhz <= 464)
  {
    write_register(CC1101_FSCTRL0, map(mhz, 378, 464, _clb[1][0], _clb[1][1]));

    if(mhz < 430.5)
    {
      write_register(CC1101_TEST0, 0x0B);
    }
    else
    {
      write_register(CC1101_TEST0, 0x09);

      uint8_t s = read_status_register(CC1101_FSCAL2);

      if(s < 32)
      {
        write_register(CC1101_FSCAL2, s + 32);
      }

      if(_last_pa != 2) set_pa(_pa);
    }
  }
  else if(mhz >= 779 && mhz <= 899.99)
  {
    write_register(CC1101_FSCTRL0, map(mhz, 779, 899, _clb[2][0], _clb[2][1]));

    if(mhz < 861)
    {
      write_register(CC1101_TEST0, 0x0B);
    }
    else
    {
      write_register(CC1101_TEST0, 0x09);

      uint8_t s = read_status_register(CC1101_FSCAL2);

      if(s < 32)
      {
        write_register(CC1101_FSCAL2, s + 32);
      }

      if(_last_pa != 3) set_pa(_pa);
    }
  }
  else if(mhz >= 900 && mhz <= 928)
  {
    write_register(CC1101_FSCTRL0, map(mhz, 900, 928, _clb[3][0], _clb[3][1]));
    write_register(CC1101_TEST0, 0x09);

    uint8_t s = read_status_register(CC1101_FSCAL2);
    
    if(s < 32)
    {
      write_register(CC1101_FSCAL2, s + 32);
    }

    if(_last_pa != 4) set_pa(_pa);
  }
}

void CC1101Switch::set_clb(uint8_t b, uint8_t s, uint8_t e)
{
  if(b < 4) 
  {
    _clb[b][0] = s;
    _clb[b][1] = e;
  }
}

void CC1101Switch::set_rxbw(uint32_t bw)
{
  _bandwidth = bw;

  float f = (float)_bandwidth;

  int s1 = 3;
  int s2 = 3;

  for(int i = 0; i < 3 && f > 101.5625f; i++)
  {
    f /= 2;
    s1--;
  }

  for(int i = 0; i < 3 && f > 58.1f; i++)
  {
    f /= 1.25f;
    s2--;
  }

  split_MDMCFG4();

  _m4RxBw = (s1 << 6) | (s2 << 4);

  write_register(CC1101_MDMCFG4, _m4RxBw + _m4DaRa);
}

void CC1101Switch::set_tx()
{
  ESP_LOGI(TAG, "CC1101 set_tx");
  send_cmd(CC1101_SIDLE);
  send_cmd(CC1101_STX);
  _trxstate = 1;
}

void CC1101Switch::set_rx()
{
  ESP_LOGI(TAG, "CC1101 set_rx");
  send_cmd(CC1101_SIDLE);
  send_cmd(CC1101_SRX);
  _trxstate = 2;
}

int32_t CC1101Switch::get_rssi()
{
  int32_t rssi;
  rssi = read_status_register(CC1101_RSSI);
  if(rssi >= 128) rssi -= 256;
  return (rssi / 2) - 74;
}

uint8_t CC1101Switch::get_lqi()
{
  return read_status_register(CC1101_LQI);
}

void CC1101Switch::set_sres()
{
  send_cmd(CC1101_SRES);
  _trxstate = 0;
}

void CC1101Switch::set_sidle()
{
  send_cmd(CC1101_SIDLE);
  _trxstate = 0;
}

void CC1101Switch::set_sleep()
{
  _trxstate = 0;
  send_cmd(CC1101_SIDLE); // Exit RX / TX, turn off frequency synthesizer and exit
  send_cmd(CC1101_SPWD); // Enter power down mode when CSn goes high.
}

void CC1101Switch::split_MDMCFG2()
{
  uint8_t calc = read_status_register(CC1101_MDMCFG2);

  _m2DCOFF = calc & 0x80;
  _m2MODFM = calc & 0x70;
  _m2MANCH = calc & 0x08;
  _m2SYNCM = calc & 0x07;
}

void CC1101Switch::split_MDMCFG4()
{
  uint8_t calc = read_status_register(CC1101_MDMCFG4);

  _m4RxBw = calc & 0xf0;
  _m4DaRa = calc & 0x0f;
}

void CC1101Switch::begin_tx()
{
  set_tx();
#ifdef USE_ESP8266
#ifdef USE_ARDUINO
  pinMode(_gdo0, OUTPUT);
  noInterrupts();
#else // USE_ESP_IDF
  gpio_set_direction((gpio_num_t)_gdo0, GPIO_MODE_OUTPUT);
  portDISABLE_INTERRUPTS()
#endif
#endif
}

void CC1101Switch::end_tx()
{
#ifdef USE_ESP8266
#ifdef USE_ARDUINO
  interrupts();
  pinMode(_gdo0, INPUT);
#else // USE_ESP_IDF
  portENABLE_INTERRUPTS()
  gpio_set_direction((gpio_num_t)_gdo0, GPIO_MODE_INPUT);
#endif
#endif
  set_rx();
  set_rx(); // yes, twice (really?)
}

} // namespace cc1101
} // namespace esphome
