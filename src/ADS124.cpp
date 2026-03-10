/**
 * ADS124.cpp
 */

#include "ADS124.hpp"

// ─────────────────────────────────────────────────────────────────────────────
//  Static multi-instance dispatch table
// ─────────────────────────────────────────────────────────────────────────────
ADS124* ADS124::_instances[ADS124_MAX_INSTANCES]  = {};
uint8_t ADS124::_instanceCount                    = 0;

void (*ADS124::_isrTable[ADS124_MAX_INSTANCES])() = {
  ADS124::_isr0,
  ADS124::_isr1,
  ADS124::_isr2,
  ADS124::_isr3,
};

void ADS124::_isr0() { if (_instances[0]) _instances[0]->_onDRDY(); }
void ADS124::_isr1() { if (_instances[1]) _instances[1]->_onDRDY(); }
void ADS124::_isr2() { if (_instances[2]) _instances[2]->_onDRDY(); }
void ADS124::_isr3() { if (_instances[3]) _instances[3]->_onDRDY(); }

void ADS124::_registerInstance() {
  if (_instanceCount < ADS124_MAX_INSTANCES) {
    _instances[_instanceCount++] = this;
    _instanceIdx = _instanceCount - 1;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────────────────────
ADS124::ADS124(const WiringConfig& wiring, DeviceConfig& config)
  : _wire(wiring), _cfg(config),
    _spiSettings(wiring.spiHz, MSBFIRST, SPI_MODE1)
{
  memset((void*)_dmaTxBuf, ADS124Cmd::NOP, sizeof(_dmaTxBuf));
  memset((void*)_dmaRxBuf, 0,              sizeof(_dmaRxBuf));
  _registerInstance();
}

// ─────────────────────────────────────────────────────────────────────────────
//  begin()
// ─────────────────────────────────────────────────────────────────────────────
bool ADS124::begin() {
  // ── GPIO setup ──────────────────────────────────────────────
  if (_wire.useCs) {
    pinMode(_wire.csPin, OUTPUT);
    _csHigh();
  }
  if (_wire.useReset && _wire.resetPin != 255) {
    pinMode(_wire.resetPin, OUTPUT);
    digitalWrite(_wire.resetPin, HIGH);   // active low, idle high
  }
  if (_wire.useStart && _wire.startPin != 255) {
    pinMode(_wire.startPin, OUTPUT);
    digitalWrite(_wire.startPin, LOW);    // low = stopped
  }
  if (_wire.useDrdy && _wire.drdyPin != 255) {
    pinMode(_wire.drdyPin, INPUT_PULLUP);
  }

  _wire.spi->begin();
  delay(2);

  // ── Reset ───────────────────────────────────────────────────
  if (_wire.useReset && _wire.resetPin != 255) {
    digitalWrite(_wire.resetPin, LOW);
    delayMicroseconds(10);
    digitalWrite(_wire.resetPin, HIGH);
  } else {
    _sendCommand(ADS124Cmd::RESET);
  }
  delay(5);   // ~4096 * tCLK at 4.096 MHz internal oscillator

  // ── Variant detection ───────────────────────────────────────
  uint8_t idReg = readRegister(ADS124Reg::ID);
  switch (idReg & 0x07) {
    case 0x0: _cfg.variant = AdsVariant::ADS124S08; break;
    case 0x1: _cfg.variant = AdsVariant::ADS124S06; break;
    case 0x4: _cfg.variant = AdsVariant::ADS114S08; break;
    case 0x5: _cfg.variant = AdsVariant::ADS114S06; break;
    default:
      Serial.print("[ADS124] Unknown ID: 0x"); Serial.println(idReg, HEX);
      return false;
  }

  if (_cfg.numChannels > maxChannels()) {
    Serial.print("[ADS124] numChannels clamped to "); Serial.println(maxChannels());
    _cfg.numChannels = maxChannels();
  }

  // ── Initial register block: INPMUX, PGA, DATARATE, REF ──────
  const ChannelConfig& ch0 = _cfg.channels[0];
  _cachedPGA               = _buildPGA(ch0.gain, ch0.pgaEnabled);

  uint8_t block[4] = {
    _buildINPMUX(ch0.posInput, ch0.negInput),
    _cachedPGA,
    _buildDATARATE(_cfg.filter, _cfg.sampleRate),
    _buildREF()
  };
  writeRegisters(ADS124Reg::INPMUX, block, 4);
  writeRegister(ADS124Reg::STATUS, 0x00);   // clear POR flag

  // ── Verify ──────────────────────────────────────────────────
  if (readRegister(ADS124Reg::INPMUX) != block[0]) {
    Serial.println("[ADS124] INPMUX verify failed");
    return false;
  }

  _nextChannel = 0;
  _ready       = true;

  // ── Attach DRDY interrupt ────────────────────────────────────
  if (_wire.useDrdy && _wire.drdyPin != 255 &&
      _instanceIdx < ADS124_MAX_INSTANCES) {
    attachInterrupt(digitalPinToInterrupt(_wire.drdyPin),
                    _isrTable[_instanceIdx], FALLING);
  }

  Serial.print("[ADS124] Ready — ");
  switch (_cfg.variant) {
    case AdsVariant::ADS124S08: Serial.print("ADS124S08"); break;
    case AdsVariant::ADS124S06: Serial.print("ADS124S06"); break;
    case AdsVariant::ADS114S08: Serial.print("ADS114S08"); break;
    case AdsVariant::ADS114S06: Serial.print("ADS114S06"); break;
    default: break;
  }
  Serial.print(" / "); Serial.print(resolutionBits()); Serial.print("-bit / ");
  Serial.print(_cfg.numChannels); Serial.println(" ch");
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  end()
// ─────────────────────────────────────────────────────────────────────────────
void ADS124::end() {
  stopConversions();
  if (_wire.useDrdy && _wire.drdyPin != 255) {
    detachInterrupt(digitalPinToInterrupt(_wire.drdyPin));
  }
  _sendCommand(ADS124Cmd::POWERDOWN);
  _ready = false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Conversion control
// ─────────────────────────────────────────────────────────────────────────────
void ADS124::startConversions() {
  if (!_ready) return;
  _nextChannel = 0;
  _applyChannel(0);       // make sure INPMUX matches ch0 before starting
  _converting  = true;
  _startPin(true);
}

void ADS124::stopConversions() {
  _converting = false;
  _startPin(false);
}

int32_t ADS124::readSingleShot(uint8_t chIdx) {
  if (chIdx >= _cfg.numChannels) return 0;

  bool wasConverting = _converting;
  stopConversions();
  delayMicroseconds(200);

  // Single-shot mode: set MODE bit in DATARATE
  uint8_t dr = _buildDATARATE(_cfg.filter, _cfg.sampleRate) | (1 << 5);
  writeRegister(ADS124Reg::DATARATE, dr);

  _applyChannel(chIdx);
  _startPin(true);

  if (!_waitDRDY(1000)) {
    Serial.println("[ADS124] readSingleShot timeout");
    writeRegister(ADS124Reg::DATARATE, _buildDATARATE(_cfg.filter, _cfg.sampleRate));
    if (wasConverting) startConversions();
    return 0;
  }

  // Single-shot: use RDATA because this is a random (non-continuous) read
  _wire.spi->beginTransaction(_spiSettings);
  _csLow();
  delayMicroseconds(1);
  _wire.spi->transfer(ADS124Cmd::RDATA);
  uint8_t b2 = _wire.spi->transfer(ADS124Cmd::NOP);
  uint8_t b1 = _wire.spi->transfer(ADS124Cmd::NOP);
  uint8_t b0 = (resolutionBits() == 24) ? _wire.spi->transfer(ADS124Cmd::NOP) : 0;
  delayMicroseconds(1);
  _csHigh();
  _wire.spi->endTransaction();

  int32_t raw = _assembleRaw(b2, b1, b0);

  // Restore continuous mode register
  writeRegister(ADS124Reg::DATARATE, _buildDATARATE(_cfg.filter, _cfg.sampleRate));
  if (wasConverting) startConversions();
  return raw;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Callback
// ─────────────────────────────────────────────────────────────────────────────
void ADS124::onSample(SampleCallback cb) { _callback = cb; }

// ─────────────────────────────────────────────────────────────────────────────
//  Runtime reconfiguration
// ─────────────────────────────────────────────────────────────────────────────
void ADS124::setPGA(PgaGain gain, bool pgaEnabled) {
  bool was = _converting;
  stopConversions();

  for (uint8_t i = 0; i < _cfg.numChannels; i++) {
    _cfg.channels[i].gain       = pgaEnabled ? gain : PgaGain::X1;
    _cfg.channels[i].pgaEnabled = pgaEnabled;
  }
  _cachedPGA = _buildPGA(gain, pgaEnabled);
  writeRegister(ADS124Reg::PGA, _cachedPGA);

  if (was) startConversions();
}

void ADS124::setSampleRate(AdcSPS sps, AdcFilter filter) {
  bool was = _converting;
  stopConversions();

  _cfg.sampleRate = sps;
  _cfg.filter     = filter;
  writeRegister(ADS124Reg::DATARATE, _buildDATARATE(filter, sps));

  if (was) startConversions();
}

void ADS124::setChannels(const ChannelConfig* channels, uint8_t count) {
  if (!count || count > ADS124_MAX_CHANNELS) return;
  bool was = _converting;
  stopConversions();

  _cfg.numChannels = count;
  for (uint8_t i = 0; i < count; i++) _cfg.channels[i] = channels[i];
  _nextChannel = 0;
  _applyChannel(0);

  if (was) startConversions();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Register access
// ─────────────────────────────────────────────────────────────────────────────
uint8_t ADS124::readRegister(uint8_t addr) {
  uint8_t v; readRegisters(addr, &v, 1); return v;
}
void ADS124::writeRegister(uint8_t addr, uint8_t val) {
  writeRegisters(addr, &val, 1);
}
void ADS124::readRegisters(uint8_t startAddr, uint8_t* buf, uint8_t count) {
  _wire.spi->beginTransaction(_spiSettings);
  _csLow(); delayMicroseconds(1);
  _wire.spi->transfer(ADS124Cmd::RREG | (startAddr & 0x1F));
  _wire.spi->transfer((count - 1) & 0x1F);
  for (uint8_t i = 0; i < count; i++) buf[i] = _wire.spi->transfer(ADS124Cmd::NOP);
  delayMicroseconds(1); _csHigh();
  _wire.spi->endTransaction();
}
void ADS124::writeRegisters(uint8_t startAddr, const uint8_t* data, uint8_t count) {
  _wire.spi->beginTransaction(_spiSettings);
  _csLow(); delayMicroseconds(1);
  _wire.spi->transfer(ADS124Cmd::WREG | (startAddr & 0x1F));
  _wire.spi->transfer((count - 1) & 0x1F);
  for (uint8_t i = 0; i < count; i++) _wire.spi->transfer(data[i]);
  delayMicroseconds(1); _csHigh();
  _wire.spi->endTransaction();
  delayMicroseconds(5);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Calibration
// ─────────────────────────────────────────────────────────────────────────────
void ADS124::calibrateSelfOffset() {
  bool was = _converting; stopConversions();
  _sendCommand(ADS124Cmd::SFOCAL); delay(500);
  if (was) startConversions();
}
void ADS124::calibrateSystemOffset() {
  bool was = _converting; stopConversions();
  _sendCommand(ADS124Cmd::SYOCAL); delay(500);
  if (was) startConversions();
}
void ADS124::calibrateSystemGain() {
  bool was = _converting; stopConversions();
  _sendCommand(ADS124Cmd::SYGCAL); delay(500);
  if (was) startConversions();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Accessors
// ─────────────────────────────────────────────────────────────────────────────
int32_t    ADS124::lastRaw    (uint8_t ch) const { return (ch < ADS124_MAX_CHANNELS) ? _lastRaw[ch]   : 0;    }
float      ADS124::lastVolts  (uint8_t ch) const { return (ch < ADS124_MAX_CHANNELS) ? _lastVolts[ch] : 0.0f; }
uint32_t   ADS124::sampleCount(uint8_t ch) const { return (ch < ADS124_MAX_CHANNELS) ? _sampleCnt[ch] : 0;    }
AdsVariant ADS124::variant    ()           const { return _cfg.variant; }
bool       ADS124::isReady    ()           const { return _ready; }

uint8_t ADS124::resolutionBits() const {
  return (_cfg.variant == AdsVariant::ADS114S08 ||
          _cfg.variant == AdsVariant::ADS114S06) ? 16 : 24;
}
uint8_t ADS124::maxChannels() const {
  return (_cfg.variant == AdsVariant::ADS124S08 ||
          _cfg.variant == AdsVariant::ADS114S08) ? 12 : 6;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private — SPI helpers
// ─────────────────────────────────────────────────────────────────────────────
void ADS124::_csLow()  { if (_wire.useCs) digitalWriteFast(_wire.csPin, LOW);  }
void ADS124::_csHigh() { if (_wire.useCs) digitalWriteFast(_wire.csPin, HIGH); }

void ADS124::_sendCommand(uint8_t cmd) {
  _wire.spi->beginTransaction(_spiSettings);
  _csLow(); delayMicroseconds(1);
  _wire.spi->transfer(cmd);
  delayMicroseconds(1); _csHigh();
  _wire.spi->endTransaction();
  delayMicroseconds(2);
}

void ADS124::_startPin(bool active) {
  if (_wire.useStart && _wire.startPin != 255) {
    digitalWriteFast(_wire.startPin, active ? HIGH : LOW);
    delayMicroseconds(2);
  } else {
    _sendCommand(active ? ADS124Cmd::START : ADS124Cmd::STOP);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private — Register builders
// ─────────────────────────────────────────────────────────────────────────────
uint8_t ADS124::_buildINPMUX(AdcChannel pos, AdcChannel neg) const {
  return ((uint8_t)pos << 4) | (uint8_t)neg;
}

uint8_t ADS124::_buildPGA(PgaGain gain, bool pgaEnabled, uint8_t delaySel) const {
  if (!pgaEnabled) gain = PgaGain::X1;
  uint8_t pgaEn = pgaEnabled ? 0x01 : 0x00;
  return ((delaySel & 0x07) << 5) | ((pgaEn & 0x03) << 3) | ((uint8_t)gain & 0x07);
}

uint8_t ADS124::_buildDATARATE(AdcFilter filter, AdcSPS sps) const {
  // G_CHOP=0, CLK=0 (internal), MODE=0 (continuous)
  return ((uint8_t)filter << 4) | ((uint8_t)sps & 0x0F);
}

uint8_t ADS124::_buildREF() const {
  uint8_t refsel = (uint8_t)_cfg.reference & 0x03;
  if (_cfg.reference == AdcRef::INTERNAL) {
    // Buffers off (internal ref close to rail), REFCON=10 (always on)
    return (refsel << 2) | 0x02;
  } else {
    // External ref: enable both input buffers
    return (1 << 5) | (1 << 4) | (refsel << 2);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private — Channel management
// ─────────────────────────────────────────────────────────────────────────────
void ADS124::_applyChannel(uint8_t chIdx) {
  const ChannelConfig& ch = _cfg.channels[chIdx];
  uint8_t newPGA          = _buildPGA(ch.gain, ch.pgaEnabled);

  if (newPGA != _cachedPGA) {
    uint8_t regs[2] = { _buildINPMUX(ch.posInput, ch.negInput), newPGA };
    writeRegisters(ADS124Reg::INPMUX, regs, 2);
    _cachedPGA = newPGA;
  } else {
    writeRegister(ADS124Reg::INPMUX, _buildINPMUX(ch.posInput, ch.negInput));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private — Core continuous read + channel switch (called from DRDY ISR)
//
//  In continuous mode the ADC drives data on DOUT/MISO immediately after
//  CS falls — the data is ALREADY being clocked out. No RDATA command is
//  sent. Instead, we use the full-duplex nature of SPI: while the ADC
//  sends data on MISO we simultaneously send a WREG command on MOSI to
//  reconfigure INPMUX (and optionally PGA) for the NEXT channel.
//
//  24-bit device, channel switch needed:
//    Byte 0  MOSI: WREG | REG_INPMUX     MISO: result MSB
//    Byte 1  MOSI: count (0x00 or 0x01)  MISO: result MID
//    Byte 2  MOSI: nextINPMUX            MISO: result LSB
//   [Byte 3  MOSI: nextPGA]              (only when gain changes)
//
//  16-bit device:
//    Byte 0  MOSI: WREG | REG_INPMUX     MISO: result MSB
//    Byte 1  MOSI: count (0x00 or 0x01)  MISO: result LSB
//   [Byte 2  MOSI: nextINPMUX]
//   [Byte 3  MOSI: nextPGA]
//
//  No channel switch (single channel or same INPMUX):
//    Bytes 0-2 (or 0-1): MOSI all NOP, MISO: result bytes
//
//  The WREG is decoded by the ADC as each byte is received. After the
//  INPMUX byte is latched the digital filter resets and the new conversion
//  begins immediately — no STOP/START required.
// ─────────────────────────────────────────────────────────────────────────────
void ADS124::_onDRDY() {
  if (!_converting) return;

  const uint8_t completedIdx = _nextChannel;
  const uint8_t nextIdx      = (_nextChannel + 1) % _cfg.numChannels;
  const bool    doSwitch     = (_cfg.numChannels > 1);
  const uint8_t dataBytes    = (resolutionBits() == 24) ? 3 : 2;

  const ChannelConfig& nextCh = _cfg.channels[nextIdx];
  const uint8_t nextINPMUX = _buildINPMUX(nextCh.posInput, nextCh.negInput);
  const uint8_t nextPGA    = _buildPGA(nextCh.gain, nextCh.pgaEnabled);
  const bool    pgaChanges = doSwitch && (nextPGA != _cachedPGA);

  // Build TX buffer
  // Slot 0..dataBytes-1 carry the WREG command overlaid on the data bytes.
  // If no switch is needed these stay as NOP (already initialised to 0x00).
  uint8_t txLen = dataBytes;
  if (doSwitch) {
    // WREG command fits inside the data window (same number of bytes):
    //   data byte 0 slot → WREG cmd
    //   data byte 1 slot → count byte
    //   data byte 2 slot → nextINPMUX  (for 24-bit; for 16-bit this is byte 2, extra)
    //  [extra byte]      → nextPGA if needed
    _dmaTxBuf[0] = ADS124Cmd::WREG | (ADS124Reg::INPMUX & 0x1F);
    _dmaTxBuf[1] = pgaChanges ? 0x01 : 0x00;   // count: 1 = write 2 regs
    _dmaTxBuf[2] = nextINPMUX;
    if (pgaChanges) {
      _dmaTxBuf[3] = nextPGA;
      txLen = (dataBytes == 3) ? 4 : 4;  // always 4 when PGA changes
    } else {
      // For 24-bit: WREG fits in exactly 3 bytes (same as data window)
      // For 16-bit: WREG needs one extra byte beyond the 2 data bytes
      txLen = (dataBytes == 3) ? 3 : 3;
    }
  } else {
    // No switch: keep TX all NOP
    _dmaTxBuf[0] = ADS124Cmd::NOP;
    _dmaTxBuf[1] = ADS124Cmd::NOP;
    _dmaTxBuf[2] = ADS124Cmd::NOP;
    _dmaTxBuf[3] = ADS124Cmd::NOP;
    txLen = dataBytes;
  }

  // ── SPI DMA transfer ─────────────────────────────────────────
  // SPI.transfer(txBuf, rxBuf, count) on Teensy 4.x uses DMA internally
  // when count is large enough; for 3-4 bytes it may fall back to PIO,
  // which is fine — the API is identical either way.
  _wire.spi->beginTransaction(_spiSettings);
  _csLow();
  delayMicroseconds(1);

  _wire.spi->transfer((void*)_dmaTxBuf, (void*)_dmaRxBuf, txLen);

  delayMicroseconds(1);
  _csHigh();
  _wire.spi->endTransaction();

  // ── Assemble result ───────────────────────────────────────────
  // _dmaRxBuf[0..dataBytes-1] contains the ADC result.
  // (MISO drives data from the first SCLK edge after CS falls,
  //  regardless of what we sent on MOSI.)
  int32_t raw;
  if (dataBytes == 3) {
    raw = _assembleRaw(_dmaRxBuf[0], _dmaRxBuf[1], _dmaRxBuf[2]);
  } else {
    // 16-bit: MSB in byte 0, LSB in byte 1
    raw = _assembleRaw(_dmaRxBuf[0], _dmaRxBuf[1], 0);
  }

  float v = _rawToVolts(raw, _cfg.channels[completedIdx].gain);

  _lastRaw  [completedIdx] = raw;
  _lastVolts[completedIdx] = v;
  _sampleCnt[completedIdx]++;

  // Advance channel state
  if (doSwitch) {
    _nextChannel = nextIdx;
    if (pgaChanges) _cachedPGA = nextPGA;
  }

  // Fire user callback
  if (_callback) _callback(completedIdx, raw, v);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private — Data helpers
// ─────────────────────────────────────────────────────────────────────────────
int32_t ADS124::_assembleRaw(uint8_t b2, uint8_t b1, uint8_t b0) const {
  if (resolutionBits() == 16) {
    int32_t raw = ((uint32_t)b2 << 8) | b1;
    if (raw & 0x8000) raw |= 0xFFFF0000;
    return raw;
  }
  int32_t raw = ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
  if (raw & 0x800000) raw |= 0xFF000000;
  return raw;
}

float ADS124::_rawToVolts(int32_t raw, PgaGain gain) const {
  float gainF     = (float)(1 << (uint8_t)gain);
  float fullScale = (resolutionBits() == 24) ? 8388608.0f : 32768.0f;
  return ((float)raw / fullScale) * (_cfg.vref / gainF);
}

bool ADS124::_waitDRDY(uint32_t timeoutMs) const {
  if (!_wire.useDrdy || _wire.drdyPin == 255) {
    delay(min(timeoutMs, (uint32_t)400));
    return true;
  }
  uint32_t t = millis();
  while (digitalReadFast(_wire.drdyPin) == HIGH) {
    if ((millis() - t) > timeoutMs) return false;
    yield();
  }
  return true;
}