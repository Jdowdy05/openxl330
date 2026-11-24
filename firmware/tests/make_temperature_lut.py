"""Generate the external-temperature lookup (`temperature_lut[165]` in firmware/sensing.c) from an NTC model.

  python3 -B tests/make_temperature_lut.py                          # print the C initializer
  python3 -B tests/make_temperature_lut.py --check firmware/sensing.c  # test_native.sh: the table must match

Model. Every number is stated here; nothing is read from a device or a vendor file.
- NTC thermistor to ground: R25 = 10 kOhm, Beta = 3377.4 K, so 1/T = 1/298.15 K + ln(R/R25)/Beta.
- Pull-up to the ADC reference: 1.005 x R25 (10.05 kOhm). The reading is ratiometric 12-bit, divider ratio
  x = R/(R + R_pullup).
- Indexing: sensing.c uses index = (raw + 200) >> 4 (16 counts per entry, clamped to 164). Entry i is evaluated at
  x = (16*i + 1.5)/4096, so it includes the firmware's fixed +200-count index shift. That shift is kept so readings
  and the 65 C trip stay where they were.
- Output: the nearest whole degree C, capped at 125. Entry 164 (about 10 C) is the cold clamp.

Origin of the parameters:
- Beta, the pull-up ratio and the 1.5-count evaluation point are a minimax fit to the temperature the servo reports for
  each ADC reading. No reference-thermometer data was used.
- Fit quality: within 0.494 C of the reported whole-degree values over the reachable range (indices 12..164). The
  mean absolute difference is about 0.25 C, the rounding of whole-degree values.
- A full Steinhart-Hart form (A + B ln R + C ln^2 R + D ln^3 R) fits no better: minimax 0.486 C.
- With the default 65 C limit, the temperature shutoff trips at index <= 53 (raw <= 663).
"""
import math, re, sys

R25_OHM, PULLUP_OHM, BETA_K, T25_K = 10000.0, 10050.0, 3377.4, 298.15
ENTRIES, COUNTS_PER_ENTRY, EVAL_OFFSET, FULL_SCALE, CAP_C = 165, 16, 1.5, 4096, 125


def celsius(i):
    """Model temperature for table entry i (unrounded)."""
    x = (COUNTS_PER_ENTRY * i + EVAL_OFFSET) / FULL_SCALE
    r = PULLUP_OHM * x / (1.0 - x)
    return 1.0 / (1.0 / T25_K + math.log(r / R25_OHM) / BETA_K) - 273.15


def table():
    return [min(CAP_C, int(math.floor(celsius(i) + 0.5))) for i in range(ENTRIES)]


def c_initializer(values, per_line=15):
    rows = [','.join(str(v) for v in values[k:k + per_line]) for k in range(0, len(values), per_line)]
    return '{\n    ' + ',\n    '.join(rows) + '\n}'


def main(argv):
    want = table()
    if len(argv) == 3 and argv[1] == '--check':
        src = open(argv[2]).read()
        m = re.search(r'temperature_lut\[165\]\s*=\s*\{(.*?)\};', src, re.S)
        got = [int(v) for v in re.findall(r'\d+', m.group(1))] if m else None
        if got != want:
            print('temperature LUT: firmware/sensing.c differs from the NTC model; regenerate with tests/make_temperature_lut.py')
            return 1
        trip = max(i for i in range(12, ENTRIES) if want[i] >= 65)
        print(f'temperature LUT = NTC model (10k, Beta {BETA_K:.1f} K, {PULLUP_OHM / 1000:g}k pull-up), 65 C at index <= {trip} '
              f'(raw <= {COUNTS_PER_ENTRY * trip - 185}) PASS')
        return 0
    if len(argv) != 1:
        print(__doc__)
        return 2
    print(f'static const uint8_t temperature_lut[{ENTRIES}] = {c_initializer(want)};')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
