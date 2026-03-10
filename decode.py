
"""
decode_ads124.py — Decode ADS124 binary log files produced by DualADC_SD.ino

Record format (14 bytes, little-endian, packed):
    uint32_t  timestamp_us   offset 0
    uint8_t   adc_idx        offset 4
    uint8_t   channel_idx    offset 5
    int32_t   raw            offset 6
    float     volts          offset 10

Usage:
    python decode_ads124.py DATA0000.BIN
    python decode_ads124.py DATA0000.BIN --output results.csv
    python decode_ads124.py DATA0000.BIN --plot
    python decode_ads124.py DATA0000.BIN --output results.csv --plot --summary
"""

import argparse
import csv
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
#  Record definition — must match the C struct exactly
# ─────────────────────────────────────────────────────────────────────────────
RECORD_FORMAT  = "<IBBif"   # little-endian: uint32 uint8 uint8 int32 float
RECORD_SIZE    = struct.calcsize(RECORD_FORMAT)  # should be 14
assert RECORD_SIZE == 14, f"Unexpected record size {RECORD_SIZE}"

@dataclass
class Record:
    timestamp_us: int
    adc_idx:      int
    channel_idx:  int
    raw:          int
    volts:        float

# ─────────────────────────────────────────────────────────────────────────────
#  Sensor configuration — mirrors DualADC_SD.ino
#  Keyed by (adc_idx, channel_idx)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class SensorConfig:
    label:          str
    unit:           str
    convert:        object          # callable(volts: float) -> (value: float, unit: str)
    gain:           int   = 1
    pga_enabled:    bool  = False
    notes:          str   = ""

def _pressure_convert(volts: float):
    """0–5 V transducer scaled to 0–2.5 V by a 2:1 resistor divider."""
    sensor_v = volts * 2.0          # undo the resistor divider
    return sensor_v, "V (sensor)"

def _load_cell_convert(volts: float):
    """
    Full-bridge load cell.  volts already has PGA gain divided out.
    Return raw differential voltage; calibrate with known weights in post.
    """
    return volts * 1000.0, "mV"    # display in mV for readability

def _thermocouple_convert(volts: float, cold_junction_c: float = 0.0):
    """
    K-type thermocouple: ~41 µV/°C sensitivity.
    volts = (Vdiff / Vref) * Vref / gain  — gain already divided out.
    Cold-junction compensation must be applied externally.
    """
    delta_c = volts / 41e-6
    return delta_c + cold_junction_c, "°C"

def _junction_temp_convert(volts: float):
    """
    TMP36 linear sensor: 10 mV/°C, 0.5 V offset at 0 °C.
    Signal divided by 2 before ADC input.
    """
    sensor_v = volts * 2.0
    temp_c   = (sensor_v - 0.5) / 0.01
    return temp_c, "°C"

# Sensor map — edit to match your hardware
SENSOR_MAP: dict[tuple[int, int], SensorConfig] = {
    (0, 0): SensorConfig(
        label       = "LoadCell",
        unit        = "mV",
        gain        = 128,
        pga_enabled = True,
        convert     = _load_cell_convert,
        notes       = "Full-bridge strain gauge. Zero tare offset in post.",
    ),
    (0, 1): SensorConfig(
        label       = "Pressure0",
        unit        = "V",
        gain        = 1,
        pga_enabled = False,
        convert     = _pressure_convert,
        notes       = "0–5 V pressure transducer scaled through 2:1 divider.",
    ),
    (1, 0): SensorConfig(
        label       = "Pressure1",
        unit        = "V",
        gain        = 1,
        pga_enabled = False,
        convert     = _pressure_convert,
        notes       = "0–5 V pressure transducer scaled through 2:1 divider.",
    ),
    (1, 1): SensorConfig(
        label       = "TC1",
        unit        = "°C",
        gain        = 64,
        pga_enabled = True,
        convert     = _thermocouple_convert,
        notes       = "K-type thermocouple #1. Cold-junction correction applied from JunctionTemp.",
    ),
    (1, 2): SensorConfig(
        label       = "TC2",
        unit        = "°C",
        gain        = 64,
        pga_enabled = True,
        convert     = _thermocouple_convert,
        notes       = "K-type thermocouple #2. Cold-junction correction applied from JunctionTemp.",
    ),
    (1, 3): SensorConfig(
        label       = "JunctionTemp",
        unit        = "°C",
        gain        = 1,
        pga_enabled = False,
        convert     = _junction_temp_convert,
        notes       = "TMP36 cold-junction reference (10 mV/°C, 0.5 V @ 0 °C).",
    ),
}

# ─────────────────────────────────────────────────────────────────────────────
#  Decoding
# ─────────────────────────────────────────────────────────────────────────────

def read_records(path: Path) -> list[Record]:
    """Parse all complete records from a binary log file."""
    data = path.read_bytes()
    n_complete = len(data) // RECORD_SIZE
    n_trailing = len(data) %  RECORD_SIZE

    if n_trailing:
        print(f"Warning: {n_trailing} trailing byte(s) ignored "
              f"(file may be truncated).", file=sys.stderr)

    records = []
    for i in range(n_complete):
        offset = i * RECORD_SIZE
        chunk  = data[offset : offset + RECORD_SIZE]
        ts, adc, ch, raw, volts = struct.unpack(RECORD_FORMAT, chunk)
        records.append(Record(ts, adc, ch, raw, volts))

    return records


def apply_cold_junction(records: list[Record]) -> dict[tuple[int, int], float]:
    """
    Build a lookup of the most recent JunctionTemp reading for each ADC.
    Used to apply cold-junction compensation to thermocouple channels.
    Returns {adc_idx: latest_junction_temp_c}.
    """
    junction: dict[int, float] = {}
    for r in records:
        cfg = SENSOR_MAP.get((r.adc_idx, r.channel_idx))
        if cfg and cfg.label == "JunctionTemp":
            temp_c, _ = cfg.convert(r.volts)
            junction[r.adc_idx] = temp_c
    return junction


def decode_record(r: Record, junction_temps: dict[int, float]) -> dict:
    """Convert one Record to a human-readable dict."""
    cfg  = SENSOR_MAP.get((r.adc_idx, r.channel_idx))
    label = cfg.label if cfg else f"ADC{r.adc_idx}_Ch{r.channel_idx}"

    if cfg:
        # Thermocouple channels need cold-junction correction
        if cfg.label in ("TC1", "TC2"):
            cj = junction_temps.get(r.adc_idx, 0.0)
            value, unit = cfg.convert(r.volts, cj)
        else:
            value, unit = cfg.convert(r.volts)
    else:
        value, unit = r.volts, "V"

    return {
        "timestamp_us":  r.timestamp_us,
        "timestamp_s":   r.timestamp_us / 1e6,
        "adc":           r.adc_idx,
        "channel":       r.channel_idx,
        "label":         label,
        "raw":           r.raw,
        "volts":         r.volts,
        "value":         value,
        "unit":          unit,
    }

# ─────────────────────────────────────────────────────────────────────────────
#  Summary statistics
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ChannelStats:
    label:   str
    unit:    str
    count:   int   = 0
    minimum: float = float("inf")
    maximum: float = float("-inf")
    total:   float = 0.0
    values:  list  = field(default_factory=list)

    def update(self, v: float) -> None:
        self.count  += 1
        self.total  += v
        self.minimum = min(self.minimum, v)
        self.maximum = max(self.maximum, v)
        self.values.append(v)

    @property
    def mean(self) -> float:
        return self.total / self.count if self.count else 0.0

    @property
    def stddev(self) -> float:
        if self.count < 2:
            return 0.0
        m = self.mean
        return (sum((x - m) ** 2 for x in self.values) / (self.count - 1)) ** 0.5


def compute_stats(decoded: list[dict]) -> dict[str, ChannelStats]:
    stats: dict[str, ChannelStats] = {}
    for row in decoded:
        key = f"ADC{row['adc']}_Ch{row['channel']}_{row['label']}"
        if key not in stats:
            stats[key] = ChannelStats(label=row["label"], unit=row["unit"])
        stats[key].update(row["value"])
    return stats


def print_summary(stats: dict[str, ChannelStats], duration_s: float) -> None:
    print("\n" + "═" * 74)
    print(f"  SUMMARY   ({duration_s:.3f} s of data)")
    print("═" * 74)
    header = f"{'Channel':<22} {'N':>7} {'Min':>12} {'Max':>12} {'Mean':>12} {'StdDev':>12}"
    print(header)
    print("─" * 74)
    for key, s in sorted(stats.items()):
        print(
            f"{s.label:<22} {s.count:>7} "
            f"{s.minimum:>11.4f}{s.unit[0]:1} "
            f"{s.maximum:>11.4f}{s.unit[0]:1} "
            f"{s.mean:>11.4f}{s.unit[0]:1} "
            f"{s.stddev:>11.4f}{s.unit[0]:1}"
        )
    print("═" * 74)


def print_sample_rate(stats: dict[str, ChannelStats], duration_s: float) -> None:
    print("\nEffective sample rates:")
    for key, s in sorted(stats.items()):
        rate = s.count / duration_s if duration_s > 0 else 0
        print(f"  {s.label:<22}  {s.count:>6} samples  →  {rate:>8.2f} SPS")

# ─────────────────────────────────────────────────────────────────────────────
#  CSV export
# ─────────────────────────────────────────────────────────────────────────────

CSV_FIELDS = [
    "timestamp_us", "timestamp_s", "adc", "channel",
    "label", "raw", "volts", "value", "unit",
]

def write_csv(decoded: list[dict], output_path: Path) -> None:
    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(decoded)
    print(f"CSV written: {output_path}  ({len(decoded)} rows)")

# ─────────────────────────────────────────────────────────────────────────────
#  Plotting (optional — requires matplotlib)
# ─────────────────────────────────────────────────────────────────────────────

def plot_data(decoded: list[dict]) -> None:
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("matplotlib not installed. Run: pip install matplotlib", file=sys.stderr)
        return

    # Group by label
    groups: dict[str, dict] = {}
    for row in decoded:
        key = row["label"]
        if key not in groups:
            groups[key] = {"t": [], "v": [], "unit": row["unit"]}
        groups[key]["t"].append(row["timestamp_s"])
        groups[key]["v"].append(row["value"])

    n = len(groups)
    if n == 0:
        return

    fig = plt.figure(figsize=(14, 3 * n), constrained_layout=True)
    fig.suptitle("ADS124 Sensor Log", fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(n, 1, figure=fig)

    for idx, (label, data) in enumerate(sorted(groups.items())):
        ax = fig.add_subplot(gs[idx])
        ax.plot(data["t"], data["v"], linewidth=0.8, color=f"C{idx}")
        ax.set_ylabel(f"{label}\n({data['unit']})", fontsize=9)
        ax.set_xlabel("Time (s)" if idx == n - 1 else "")
        ax.grid(True, alpha=0.3)
        ax.margins(x=0.01)

        # Annotate min/max
        vv = data["v"]
        tt = data["t"]
        if vv:
            mn, mx = min(vv), max(vv)
            ax.axhline(mn, color="steelblue", linewidth=0.5, linestyle="--", alpha=0.6)
            ax.axhline(mx, color="tomato",    linewidth=0.5, linestyle="--", alpha=0.6)
            ax.annotate(f"min={mn:.4g} {data['unit']}",
                        xy=(tt[vv.index(mn)], mn),
                        xytext=(8, 4), textcoords="offset points",
                        fontsize=7, color="steelblue")
            ax.annotate(f"max={mx:.4g} {data['unit']}",
                        xy=(tt[vv.index(mx)], mx),
                        xytext=(8, -10), textcoords="offset points",
                        fontsize=7, color="tomato")

    plt.show()

# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Decode ADS124 binary log files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("input",
                   type=Path,
                   help="Binary log file (e.g. DATA0000.BIN)")
    p.add_argument("-o", "--output",
                   type=Path,
                   default=None,
                   metavar="FILE.csv",
                   help="Write decoded data to a CSV file")
    p.add_argument("--plot",
                   action="store_true",
                   help="Plot all channels with matplotlib")
    p.add_argument("--summary",
                   action="store_true",
                   help="Print per-channel statistics")
    p.add_argument("--print-records",
                   action="store_true",
                   help="Print every decoded record to stdout")
    p.add_argument("--max-print",
                   type=int,
                   default=200,
                   metavar="N",
                   help="Max records to print (default 200; 0 = unlimited)")
    return p


def main() -> None:
    args = build_parser().parse_args()

    input_path: Path = args.input
    if not input_path.exists():
        sys.exit(f"File not found: {input_path}")

    # ── Read ────────────────────────────────────────────────────
    records = read_records(input_path)
    if not records:
        sys.exit("No complete records found in file.")

    print(f"File   : {input_path}  ({input_path.stat().st_size} bytes)")
    print(f"Records: {len(records)}")
    print(f"Format : {RECORD_SIZE} bytes/record")

    # ── Cold-junction pass ───────────────────────────────────────
    # Build a time-ordered map of junction temperatures so thermocouple
    # channels can be corrected with the nearest preceding measurement.
    # Simple approach: use one global latest value per ADC.
    junction_temps = apply_cold_junction(records)
    if junction_temps:
        for adc_i, t in junction_temps.items():
            print(f"Cold junction ADC{adc_i}: {t:.2f} °C (last reading)")

    # ── Decode all records ───────────────────────────────────────
    decoded = [decode_record(r, junction_temps) for r in records]

    # Duration
    if len(records) > 1:
        duration_us = records[-1].timestamp_us - records[0].timestamp_us
        duration_s  = duration_us / 1e6
    else:
        duration_s = 0.0
    print(f"Duration: {duration_s:.3f} s")

    # ── Print records ────────────────────────────────────────────
    if args.print_records:
        limit = args.max_print if args.max_print > 0 else len(decoded)
        truncated = len(decoded) > limit

        col = f"{'Time(s)':>10}  {'ADC':>3}  {'Ch':>2}  {'Label':<14}  " \
              f"{'Raw':>10}  {'Volts':>12}  {'Value':>12}  Unit"
        print("\n" + col)
        print("─" * len(col))

        for row in decoded[:limit]:
            print(
                f"{row['timestamp_s']:>10.4f}  "
                f"{row['adc']:>3}  "
                f"{row['channel']:>2}  "
                f"{row['label']:<14}  "
                f"{row['raw']:>10}  "
                f"{row['volts']:>12.6f}  "
                f"{row['value']:>12.4f}  "
                f"{row['unit']}"
            )
        if truncated:
            print(f"  ... ({len(decoded) - limit} more records; "
                  f"use --max-print 0 to show all)")

    # ── Summary ──────────────────────────────────────────────────
    if args.summary:
        stats = compute_stats(decoded)
        print_summary(stats, duration_s)
        print_sample_rate(stats, duration_s)

    # ── CSV export ───────────────────────────────────────────────
    if args.output:
        write_csv(decoded, args.output)

    # ── Plot ─────────────────────────────────────────────────────
    if args.plot:
        plot_data(decoded)


if __name__ == "__main__":
    main()