/**
 * ADS124.h — Driver for Texas Instruments ADS124S0x / ADS114S0x ADCs
 *
 * Supports:
 *   ADS124S08  24-bit, 12-channel
 *   ADS124S06  24-bit,  6-channel
 *   ADS114S08  16-bit, 12-channel
 *   ADS114S06  16-bit,  6-channel
 *
 * Continuous conversion read (correct datasheet behaviour):
 *   DRDY falls → CS low → clock out 3 bytes (data arrives on MISO
 *   immediately — no RDATA command needed) → embed WREG on MOSI at the
 *   same time to switch channel for next conversion → CS high.
 *   RDATA is only used for single-shot / random reads.
 *
 * Multi-instance support:
 *   Up to ADS124_MAX_INSTANCES devices on separate CS/DRDY pins.
 *   Each instance registers itself; the static ISR dispatch table
 *   routes each falling-edge interrupt to the correct object.
 *
 * DMA:
 *   Uses Teensy 4.x SPI.transfer(txBuf, rxBuf, count) DMA bulk
 *   transfer for the 3-byte read + optional WREG bytes in one
 *   uninterrupted CS assertion.
 */

#pragma once
#include <Arduino.h>
#include <SPI.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Compile-time limits
// ─────────────────────────────────────────────────────────────────────────────
#ifndef ADS124_MAX_CHANNELS
  #define ADS124_MAX_CHANNELS 12
#endif

#ifndef ADS124_MAX_INSTANCES
  #define ADS124_MAX_INSTANCES 4
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  Register addresses
// ─────────────────────────────────────────────────────────────────────────────
namespace ADS124Reg {
  constexpr uint8_t ID       = 0x00;
  constexpr uint8_t STATUS   = 0x01;
  constexpr uint8_t INPMUX   = 0x02;
  constexpr uint8_t PGA      = 0x03;
  constexpr uint8_t DATARATE = 0x04;
  constexpr uint8_t REF      = 0x05;
  constexpr uint8_t IDACMAG  = 0x06;
  constexpr uint8_t IDACMUX  = 0x07;
  constexpr uint8_t VBIAS    = 0x08;
  constexpr uint8_t SYS      = 0x09;
  constexpr uint8_t OFCAL0   = 0x0A;
  constexpr uint8_t OFCAL1   = 0x0B;
  constexpr uint8_t OFCAL2   = 0x0C;
  constexpr uint8_t FSCAL0   = 0x0D;
  constexpr uint8_t FSCAL1   = 0x0E;
  constexpr uint8_t FSCAL2   = 0x0F;
  constexpr uint8_t GPIODAT  = 0x10;
  constexpr uint8_t GPIOCON  = 0x11;
  constexpr uint8_t COUNT    = 0x12;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Commands
// ─────────────────────────────────────────────────────────────────────────────
namespace ADS124Cmd {
  constexpr uint8_t NOP       = 0x00;
  constexpr uint8_t WAKEUP    = 0x02;
  constexpr uint8_t POWERDOWN = 0x04;
  constexpr uint8_t RESET     = 0x06;
  constexpr uint8_t START     = 0x08;
  constexpr uint8_t STOP      = 0x0A;
  constexpr uint8_t RDATA     = 0x12;
  constexpr uint8_t RREG      = 0x20;
  constexpr uint8_t WREG      = 0x40;
  constexpr uint8_t SYOCAL    = 0x16;
  constexpr uint8_t SYGCAL    = 0x17;
  constexpr uint8_t SFOCAL    = 0x19;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Enumerations
// ─────────────────────────────────────────────────────────────────────────────

enum class AdcChannel : uint8_t {
  AIN0   = 0x0, AIN1  = 0x1, AIN2  = 0x2,  AIN3  = 0x3,
  AIN4   = 0x4, AIN5  = 0x5, AIN6  = 0x6,  AIN7  = 0x7,
  AIN8   = 0x8, AIN9  = 0x9, AIN10 = 0xA,  AIN11 = 0xB,
  AINCOM = 0xC
};

enum class PgaGain : uint8_t {
  X1   = 0, X2  = 1, X4  = 2, X8   = 3,
  X16  = 4, X32 = 5, X64 = 6, X128 = 7
};

enum class AdcSPS : uint8_t {
  SPS_2_5  = 0x0, SPS_5    = 0x1, SPS_10   = 0x2, SPS_16_6 = 0x3,
  SPS_20   = 0x4, SPS_50   = 0x5, SPS_60   = 0x6, SPS_100  = 0x7,
  SPS_200  = 0x8, SPS_400  = 0x9, SPS_800  = 0xA, SPS_1000 = 0xB,
  SPS_2000 = 0xC, SPS_4000 = 0xD
};

enum class AdcFilter : uint8_t {
  SINC3       = 0,
  LOW_LATENCY = 1
};

enum class AdsVariant : uint8_t {
  ADS124S08 = 0x0,
  ADS124S06 = 0x1,
  ADS114S08 = 0x4,
  ADS114S06 = 0x5,
  UNKNOWN   = 0xFF
};

enum class AdcRef : uint8_t {
  REFP0_REFN0 = 0x0,
  REFP1_REFN1 = 0x1,
  INTERNAL    = 0x2,
};

// ─────────────────────────────────────────────────────────────────────────────
//  Configuration structures
// ─────────────────────────────────────────────────────────────────────────────

struct ChannelConfig {
  AdcChannel  posInput   = AdcChannel::AIN0;
  AdcChannel  negInput   = AdcChannel::AIN1;
  PgaGain     gain       = PgaGain::X1;
  bool        pgaEnabled = false;
  const char* label      = "";
};

struct WiringConfig {
  bool      useCs    = true;
  bool      useDrdy  = true;
  bool      useReset = false;
  bool      useStart = false;

  uint8_t   csPin    = 10;
  uint8_t   drdyPin  = 9;
  uint8_t   resetPin = 255;
  uint8_t   startPin = 255;

  SPIClass* spi      = &SPI;
  uint32_t  spiHz    = 4000000;
};

struct DeviceConfig {
  AdcSPS        sampleRate  = AdcSPS::SPS_20;
  AdcFilter     filter      = AdcFilter::LOW_LATENCY;
  AdcRef        reference   = AdcRef::INTERNAL;
  float         vref        = 2.5f;
  AdsVariant    variant     = AdsVariant::UNKNOWN;  // filled by begin()
  ChannelConfig channels[ADS124_MAX_CHANNELS];
  uint8_t       numChannels = 1;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Sample callback
//    ch    — channel index in DeviceConfig::channels[]
//    raw   — signed result, sign-extended to int32_t
//    volts — converted voltage
// ─────────────────────────────────────────────────────────────────────────────
using SampleCallback = void (*)(uint8_t ch, int32_t raw, float volts);

// ─────────────────────────────────────────────────────────────────────────────
//  ADS124 driver class
// ─────────────────────────────────────────────────────────────────────────────
class ADS124 {
public:
  explicit ADS124(const WiringConfig& wiring, DeviceConfig& config);

  // ── Lifecycle ────────────────────────────────────────────────
  bool begin();
  void end();

  // ── Conversion control ───────────────────────────────────────
  void    startConversions();
  void    stopConversions();

  /**
   * Perform a single-shot read on chIdx.
   * Stops continuous conversions, reads once via RDATA, then restores state.
   */
  int32_t readSingleShot(uint8_t chIdx);

  // ── Callback ─────────────────────────────────────────────────
  void onSample(SampleCallback cb);

  // ── Runtime reconfiguration ──────────────────────────────────
  void setPGA       (PgaGain gain, bool pgaEnabled);
  void setSampleRate(AdcSPS sps, AdcFilter filter);
  void setChannels  (const ChannelConfig* channels, uint8_t count);

  // ── Register access ──────────────────────────────────────────
  uint8_t readRegister  (uint8_t addr);
  void    writeRegister (uint8_t addr, uint8_t value);
  void    readRegisters (uint8_t startAddr, uint8_t* buf, uint8_t count);
  void    writeRegisters(uint8_t startAddr, const uint8_t* data, uint8_t count);

  // ── Calibration ──────────────────────────────────────────────
  void calibrateSelfOffset();
  void calibrateSystemOffset();
  void calibrateSystemGain();

  // ── Accessors ────────────────────────────────────────────────
  int32_t    lastRaw    (uint8_t ch) const;
  float      lastVolts  (uint8_t ch) const;
  uint32_t   sampleCount(uint8_t ch) const;
  AdsVariant variant    ()           const;
  uint8_t    resolutionBits()        const;
  uint8_t    maxChannels()           const;
  bool       isReady    ()           const;

private:
  WiringConfig  _wire;
  DeviceConfig& _cfg;
  SPISettings   _spiSettings;

  uint8_t  _nextChannel = 0;
  bool     _converting  = false;
  bool     _ready       = false;
  uint8_t  _cachedPGA   = 0;

  int32_t  _lastRaw  [ADS124_MAX_CHANNELS] = {};
  float    _lastVolts[ADS124_MAX_CHANNELS] = {};
  uint32_t _sampleCnt[ADS124_MAX_CHANNELS] = {};

  SampleCallback _callback    = nullptr;
  uint8_t        _instanceIdx = 0;     // index into _instances[] for ISR dispatch

  // DMA buffers
  // In continuous mode TX carries the WREG payload (channel switch).
  // Maximum transaction: 3 data bytes + WREG cmd + count + INPMUX + PGA = 7 bytes.
  // We size to 8 for alignment.
  static constexpr uint8_t DMA_BUF_SIZE = 8;
  volatile uint8_t _dmaTxBuf[DMA_BUF_SIZE] __attribute__((aligned(4)));
  volatile uint8_t _dmaRxBuf[DMA_BUF_SIZE] __attribute__((aligned(4)));

  // ── Multi-instance ISR dispatch ──────────────────────────────
  // Indexed by DRDY pin number for O(1) lookup from ISR.
  static ADS124* _instances[ADS124_MAX_INSTANCES];
  static uint8_t _instanceCount;

  // One static ISR per possible instance slot (Teensy requires
  // plain function pointers with no arguments for attachInterrupt).
  static void _isr0();
  static void _isr1();
  static void _isr2();
  static void _isr3();
  static void (*_isrTable[ADS124_MAX_INSTANCES])();

  void _registerInstance();

  // ── SPI helpers ──────────────────────────────────────────────
  void _csLow();
  void _csHigh();
  void _sendCommand(uint8_t cmd);
  void _startPin(bool active);

  // ── Register builders ────────────────────────────────────────
  uint8_t _buildINPMUX  (AdcChannel pos, AdcChannel neg) const;
  uint8_t _buildPGA     (PgaGain gain, bool pgaEnabled, uint8_t delaySel = 0) const;
  uint8_t _buildDATARATE(AdcFilter filter, AdcSPS sps)   const;
  uint8_t _buildREF     ()                               const;

  // ── Channel management ───────────────────────────────────────
  void _applyChannel(uint8_t chIdx);

  // ── Core continuous read + channel switch ────────────────────
  // Called from DRDY ISR.
  // In continuous mode the ADC presents data on MISO immediately
  // after CS falls — no RDATA command required.
  //
  // Full-duplex SPI transaction (24-bit device, single channel, WREG for next ch):
  //
  //   Byte 0  DIN: WREG | REG_INPMUX   DOUT: ADC result MSB
  //   Byte 1  DIN: count (0x00 / 0x01) DOUT: ADC result MID
  //   Byte 2  DIN: nextINPMUX          DOUT: ADC result LSB
  //  [Byte 3  DIN: nextPGA]            (only when gain changes)
  //
  // For a 16-bit device the result is only 2 bytes, so we send
  // the WREG command starting at byte 0 and read 2 bytes of data.
  //
  // When there is only one channel (no switch needed), DIN is all
  // NOP (0x00) and we just clock out 3 bytes.
  void _onDRDY();

  // ── Data helpers ─────────────────────────────────────────────
  int32_t _assembleRaw (uint8_t b2, uint8_t b1, uint8_t b0) const;
  float   _rawToVolts  (int32_t raw, PgaGain gain)           const;
  bool    _waitDRDY    (uint32_t timeoutMs = 500)            const;
};