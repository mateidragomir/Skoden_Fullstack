/**
 * DualADC_SD.ino
 *
 * Two ADS124S08 ADCs with DRDY-interrupt driven async reads.
 * Results are pushed into ring buffers and periodically flushed
 * to an SD card as binary records.
 *
 * ── ADC 0 (CS=10, DRDY=9) ───────────────────────────────────────
 *   Ch0  AIN0–AIN1   Load cell (full-bridge strain gauge)   PGA x128
 *   Ch1  AIN2–AINCOM Pressure transducer (0–5 V SE)         PGA off
 *
 * ── ADC 1 (CS=8, DRDY=7) ────────────────────────────────────────
 *   Ch0  AIN0–AIN1   Pressure transducer (0–5 V SE)         PGA off
 *   Ch1  AIN2–AIN3   K-type thermocouple #1                 PGA x64
 *   Ch2  AIN4–AIN5   K-type thermocouple #2                 PGA x64
 *   Ch3  AIN6–AINCOM Junction temperature sensor (SE)       PGA off
 *
 * ── Wiring ──────────────────────────────────────────────────────
 *   Teensy 4.1 Pin 10  → ADC0 CS
 *   Teensy 4.1 Pin  9  → ADC0 DRDY
 *   Teensy 4.1 Pin  8  → ADC1 CS
 *   Teensy 4.1 Pin  7  → ADC1 DRDY
 *   Teensy 4.1 Pin 13  → SCLK  (shared)
 *   Teensy 4.1 Pin 11  → MOSI  (shared)
 *   Teensy 4.1 Pin 12  → MISO  (shared, tri-stated by CS)
 *   Teensy 4.1 Pin BUILTIN_SDCARD → SD card (Teensy 4.1 built-in)
 *
 * ── SD record format ────────────────────────────────────────────
 *   Each record is fixed-size:
 *     uint32_t  timestamp_us
 *     uint8_t   adcIdx
 *     uint8_t   channelIdx
 *     int32_t   raw
 *     float     volts
 *   = 14 bytes per record
 *
 * ── Notes on sensor conditioning ────────────────────────────────
 *   Load cell:
 *     Full-bridge output typically 1–3 mV/V. With 2.5 V excitation
 *     and x128 PGA, full-scale range ≈ ±19.5 mV input.
 *     Zero the tare offset in software by subtracting baseline.
 *
 *   Pressure transducer (0–5 V output):
 *     The ADC reference is 2.5 V internal. A 0–5 V signal MUST be
 *     scaled down to 0–2.5 V with a resistor divider before the ADC
 *     input. The volts value output by the driver reflects the scaled
 *     (divided) voltage; multiply by 2 in application code to get the
 *     true transducer output, then apply the sensor's transfer function.
 *
 *   K-type thermocouple:
 *     Output ~41 µV/°C. With x64 PGA and 2.5 V ref, resolution ≈ 0.7 µV
 *     per LSB ≈ 0.017 °C per LSB. Cold-junction compensation is required;
 *     read the junction temperature sensor to get the reference temperature.
 *
 *   Junction temperature sensor (linear 0–5 V, e.g. LM35 / TMP36):
 *     Same 2:1 divider as pressure transducer above.
 */

#include <SD.h>
#include "ADS124.hpp"

// ─────────────────────────────────────────────────────────────────
//  SD card
// ─────────────────────────────────────────────────────────────────
// Teensy 4.1 has a built-in SD card slot on the dedicated SDIO bus.
// BUILTIN_SDCARD is defined by the Teensy core.
static const uint8_t SD_CS_PIN = BUILTIN_SDCARD;

// Flush to SD every N milliseconds
static const uint32_t FLUSH_INTERVAL_MS = 100;

// ─────────────────────────────────────────────────────────────────
//  SD record layout (packed, 14 bytes)
// ─────────────────────────────────────────────────────────────────
struct __attribute__((packed)) SampleRecord {
  uint32_t timestampUs;   // micros() at sample time
  uint8_t  adcIdx;
  uint8_t  channelIdx;
  int32_t  raw;
  float    volts;
};
static_assert(sizeof(SampleRecord) == 14, "SampleRecord size mismatch");

// ─────────────────────────────────────────────────────────────────
//  Ring buffer
//  Sized for worst-case: 2 ADCs × 4 channels × 4000 SPS × 0.1 s flush
//  = 3200 records. Use a power-of-2 for cheap modulo.
// ─────────────────────────────────────────────────────────────────
static const uint16_t RING_SIZE = 4096;  // must be power of 2
static const uint16_t RING_MASK = RING_SIZE - 1;

struct RingBuffer {
  SampleRecord records[RING_SIZE];
  volatile uint16_t head = 0;   // written by ISR
  uint16_t         tail = 0;   // read  by main loop

  // Push from ISR — non-blocking. Drops oldest record on overflow.
  inline void push(const SampleRecord& r) {
    uint16_t next = (head + 1) & RING_MASK;
    records[head] = r;
    head = next;
    // If head catches tail the oldest record is silently overwritten.
    // Increase RING_SIZE or decrease flush interval if this occurs.
  }

  // Number of records available to read
  inline uint16_t available() const {
    return (head - tail) & RING_MASK;
  }

  // Pop one record into r. Returns false if empty.
  inline bool pop(SampleRecord& r) {
    if (tail == head) return false;
    r    = records[tail];
    tail = (tail + 1) & RING_MASK;
    return true;
  }
};

static RingBuffer ringBuf;

// ─────────────────────────────────────────────────────────────────
//  ADC 0 configuration — load cell + pressure transducer
// ─────────────────────────────────────────────────────────────────
WiringConfig wiring0 = {
  /* useCs    */ true,
  /* useDrdy  */ true,
  /* useReset */ false,
  /* useStart */ false,
  /* csPin    */ 10,
  /* drdyPin  */ 9,
  /* resetPin */ 255,
  /* startPin */ 255,
  /* spi      */ &SPI,
  /* spiHz    */ 4000000
};

DeviceConfig config0 = {
  /* sampleRate */ AdcSPS::SPS_100,        // 100 SPS per channel; 50 SPS effective per sensor
  /* filter     */ AdcFilter::LOW_LATENCY, // single-cycle settling for multi-channel
  /* reference  */ AdcRef::INTERNAL,
  /* vref       */ 2.5f,
  /* variant    */ AdsVariant::UNKNOWN,
  /* channels   */ {
    // Ch0: Load cell — full bridge, PGA x128
    // AIN0 = EXC+, AIN1 = EXC-  (connect bridge sense lines here)
    // Adjust excitation connections to match your bridge wiring.
    {
      /* pos */ AdcChannel::AIN0,
      /* neg */ AdcChannel::AIN1,
      /* gain */ PgaGain::X128,
      /* pga  */ true,
      /* label */ "LoadCell"
    },
    // Ch1: Pressure transducer — 0–5 V single-ended
    // Signal MUST be divided to 0–2.5 V before the ADC input.
    // Use a 10k/10k resistor divider: AIN2 = junction, AGND = ground.
    {
      /* pos */ AdcChannel::AIN2,
      /* neg */ AdcChannel::AINCOM,
      /* gain */ PgaGain::X1,
      /* pga  */ false,
      /* label */ "Pressure0"
    },
  },
  /* numChannels */ 2
};

ADS124 adc0(wiring0, config0);

// // ─────────────────────────────────────────────────────────────────
// //  ADC 1 configuration — pressure, 2× thermocouple, junction temp
// // ─────────────────────────────────────────────────────────────────
// WiringConfig wiring1 = {
//   /* useCs    */ true,
//   /* useDrdy  */ true,
//   /* useReset */ false,
//   /* useStart */ false,
//   /* csPin    */ 8,
//   /* drdyPin  */ 7,
//   /* resetPin */ 255,
//   /* startPin */ 255,
//   /* spi      */ &SPI,
//   /* spiHz    */ 4000000
// };

// DeviceConfig config1 = {
//   /* sampleRate */ AdcSPS::SPS_50,
//   /* filter     */ AdcFilter::LOW_LATENCY,
//   /* reference  */ AdcRef::INTERNAL,
//   /* vref       */ 2.5f,
//   /* variant    */ AdsVariant::UNKNOWN,
//   /* channels   */ {
//     // Ch0: Pressure transducer — 0–5 V single-ended (divided to 0–2.5 V)
//     {
//       /* pos */ AdcChannel::AIN0,
//       /* neg */ AdcChannel::AINCOM,
//       /* gain */ PgaGain::X1,
//       /* pga  */ false,
//       /* label */ "Pressure1"
//     },
//     // Ch1: K-type thermocouple #1 — differential, PGA x64
//     // Thermocouple output: ~41 µV/°C.
//     // Connect + lead to AIN2, – lead to AIN3.
//     // Add cold-junction compensation using the junction temp reading (Ch3).
//     {
//       /* pos */ AdcChannel::AIN2,
//       /* neg */ AdcChannel::AIN3,
//       /* gain */ PgaGain::X64,
//       /* pga  */ true,
//       /* label */ "TC1"
//     },
//     // Ch2: K-type thermocouple #2
//     {
//       /* pos */ AdcChannel::AIN4,
//       /* neg */ AdcChannel::AIN5,
//       /* gain */ PgaGain::X64,
//       /* pga  */ true,
//       /* label */ "TC2"
//     },
//     // Ch3: Junction (cold junction) temperature sensor — linear 0–5 V SE
//     // e.g. LM35 (10 mV/°C) or TMP36 (10 mV/°C, 0.5 V at 0 °C)
//     // Divide signal to 0–2.5 V before input. Multiply volts×2 in app code.
//     {
//       /* pos */ AdcChannel::AIN6,
//       /* neg */ AdcChannel::AINCOM,
//       /* gain */ PgaGain::X1,
//       /* pga  */ false,
//       /* label */ "JunctionTemp"
//     },
//   },
//   /* numChannels */ 4
// };

// ADS124 adc1(wiring1, config1);

// ─────────────────────────────────────────────────────────────────
//  Sample callbacks — called from DRDY ISR, kept minimal
// ─────────────────────────────────────────────────────────────────
void onSample0(uint8_t ch, int32_t raw, float volts) {
  SampleRecord r;
  r.timestampUs = micros();
  r.adcIdx      = 0;
  r.channelIdx  = ch;
  r.raw         = raw;
  r.volts       = volts;
  ringBuf.push(r);
}

// void onSample1(uint8_t ch, int32_t raw, float volts) {
//   SampleRecord r;
//   r.timestampUs = micros();
//   r.adcIdx      = 1;
//   r.channelIdx  = ch;
//   r.raw         = raw;
//   r.volts       = volts;
//   ringBuf.push(r);
// }

// ─────────────────────────────────────────────────────────────────
//  SD housekeeping
// ─────────────────────────────────────────────────────────────────
static File    dataFile;
static uint32_t lastFlushMs  = 0;
static uint32_t totalWritten = 0;
static bool     sdOk         = false;

// Write a header CSV line so the binary file can be identified,
// then switch to binary for speed.
// In practice you may prefer a pure binary file; adjust as needed.
static void openDataFile() {
  char filename[32];
  // Simple incrementing filename: DATA0000.BIN .. DATA9999.BIN
  for (uint16_t i = 0; i < 10000; i++) {
    snprintf(filename, sizeof(filename), "DATA%04u.BIN", i);
    if (!SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      if (dataFile) {
        Serial.print("Logging to: "); Serial.println(filename);
      }
      return;
    }
  }
  Serial.println("ERROR: Could not create data file");
}

// Flush ring buffer to SD. Called from main loop only.
static void flushToSD() {
  if (!sdOk || !dataFile) return;

  SampleRecord r;
  uint16_t written = 0;

  // Drain all currently available records. New ones may arrive during
  // this call (head advances in ISR) — that is safe because push() is
  // lock-free for single-producer/single-consumer.
  uint16_t toWrite = ringBuf.available();
  while (written < toWrite && ringBuf.pop(r)) {
    dataFile.write((uint8_t*)&r, sizeof(SampleRecord));
    written++;
  }

  if (written > 0) {
    dataFile.flush();
    totalWritten += written;
  }
}

// ─────────────────────────────────────────────────────────────────
//  Debug print helper (Serial, not SD)
// ─────────────────────────────────────────────────────────────────
static void printSample(const char* adcName, uint8_t ch,
                         const char* label, int32_t raw, float volts) {
  Serial.print(adcName); Serial.print(" ch");
  Serial.print(ch); Serial.print(" ["); Serial.print(label); Serial.print("]  raw=");
  Serial.print(raw); Serial.print("  V=");
  Serial.println(volts, 6);
}

// ─────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("\n=== Dual ADS124S08 + SD logging ===");

  // ── SD card ─────────────────────────────────────────────────
  if (SD.begin(SD_CS_PIN)) {
    sdOk = true;
    openDataFile();
  } else {
    Serial.println("WARNING: SD card not found — data will not be saved");
  }

  // ── ADC 0 ───────────────────────────────────────────────────
  if (!adc0.begin()) {
    Serial.println("FATAL: ADC0 init failed");
    while (true) delay(1000);
  }
  adc0.onSample(onSample0);

  // ── ADC 1 ───────────────────────────────────────────────────
  // if (!adc1.begin()) {
  //   Serial.println("FATAL: ADC1 init failed");
  //   while (true) delay(1000);
  // }
  // adc1.onSample(onSample1);

  // ── Start both ADCs ─────────────────────────────────────────
  adc0.startConversions();
  // adc1.startConversions();

  Serial.println("Acquisition running. Press any key to stop.");
}

// ─────────────────────────────────────────────────────────────────
//  Loop — flush SD and print diagnostics
// ─────────────────────────────────────────────────────────────────
static uint32_t lastPrintMs = 0;

void loop() {
  // ── Flush ring buffer to SD every FLUSH_INTERVAL_MS ─────────
  uint32_t now = millis();
  if (now - lastFlushMs >= FLUSH_INTERVAL_MS) {
    lastFlushMs = now;
    flushToSD();
  }

  // ── Print last value for each sensor every 500 ms ────────────
  if (now - lastPrintMs >= 500) {
    lastPrintMs = now;

    // ADC 0
    printSample("ADC0", 0, config0.channels[0].label,
                adc0.lastRaw(0), adc0.lastVolts(0));
    // Pressure: raw ADC voltage is Vtransducer/2 (due to divider); ×2 to recover
    float pressureV = adc0.lastVolts(1) * 2.0f;
    Serial.print("ADC0 ch1 [Pressure0]  raw="); Serial.print(adc0.lastRaw(1));
    Serial.print("  Vsensor="); Serial.print(pressureV, 4); Serial.println(" V");

    // ADC 1
    // float pressureV1 = adc1.lastVolts(0) * 2.0f;
    // Serial.print("ADC1 ch0 [Pressure1]  Vsensor="); Serial.print(pressureV1, 4); Serial.println(" V");

    // Thermocouples: volts = (raw / FS) * Vref / gain
    // To get temperature: T_measured = volts / (41e-6 * gain) — but gain is
    // already divided out by rawToVolts, so:
    //   T_differential_C = adc1.lastVolts(ch) / 41e-6
    // Then add cold-junction temperature for absolute temperature.
    // float junctionV    = adc1.lastVolts(3) * 2.0f;  // ×2 for divider
    // LM35: 10 mV/°C; TMP36: 10 mV/°C offset by 0.5 V at 0 °C — adjust to your sensor
    // float junctionTemp = (junctionV - 0.5f) / 0.01f;  // TMP36 example

    // float tc1DeltaV = adc1.lastVolts(1);   // already divided by gain
    // float tc2DeltaV = adc1.lastVolts(2);
    // float tc1TempC  = (tc1DeltaV / 41e-6f) + junctionTemp;
    // float tc2TempC  = (tc2DeltaV / 41e-6f) + junctionTemp;

    // Serial.print("ADC1 ch1 [TC1]  T="); Serial.print(tc1TempC, 2); Serial.println(" °C");
    // Serial.print("ADC1 ch2 [TC2]  T="); Serial.print(tc2TempC, 2); Serial.println(" °C");
    // Serial.print("ADC1 ch3 [Junction]  T="); Serial.print(junctionTemp, 2); Serial.println(" °C");

    Serial.print("Total records written: "); Serial.println(totalWritten);
    Serial.print("Ring buffer available: "); Serial.println(ringBuf.available());
    Serial.println("---");
  }

  // ── Stop on any serial input ─────────────────────────────────
  if (Serial.available()) {
    adc0.stopConversions();
    // adc1.stopConversions();
    if (dataFile) dataFile.close();
    Serial.print("Stopped. Total records: "); Serial.println(totalWritten);
    while (true) delay(1000);
  }
}

int main() {
    setup();
    while (true) {
        loop();
    }
}