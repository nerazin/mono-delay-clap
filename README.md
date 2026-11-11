# Mono Delay

The per-track delay of the Elektron Monomachine as a CLAP effect for Bitwig
(macOS, universal binary).

The DSP is a fixed-point C port of the delay in Monomachine OS 1.32B, written
from its DSP56300 code and checked sample for sample against that code running
in an emulator. It reproduces the delay's timing, how it reacts when you move
the time, and the filter in the feedback loop. No firmware code or data is in
this repository: the tables are generated from formulas at start-up.

## Controls

| Parameter    | Range     | On the Monomachine | What it does |
|--------------|-----------|--------------------|--------------|
| Time         | 0–127     | DTIM | (Time + 1) / 256 of a whole note: 15 = 1/16, 31 = 1/8, 47 = dotted 1/8, 63 = 1/4, 127 = 1/2. |
| Feedback     | 0–127     | DFB  | Loop gain is Feedback / 64: above 64 the echoes grow until they saturate, as on the unit. |
| Filter Base  | 0–127     | DBAS | One-pole high-pass in the loop, 10 Hz to 18.8 kHz. |
| Filter Width | 0–127     | DWID | One-pole low-pass at Base + Width (capped at 127). |
| Ping-Pong    | Off / On  | sign of DSND | On: echoes alternate between left and right. |
| Mix          | 0–100 %   | (replaces DSND) | 0 % dry only, 100 % delay only. The input always goes into the delay at full level. |
| Tempo Sync   | On / Off  | (none) | On (default): Time follows the host's tempo. Off: the delay runs at 120 BPM, the unit's default tempo, whatever the host does: Time 0–127 is then 7.8 ms to 1 s in 7.8 ms steps. |

All parameters can be automated and modulated in Bitwig. The Time field shows
the delay in milliseconds, as the firmware computes it at the current tempo,
and accepts a time in milliseconds (it picks the nearest step).

## How it behaves

This is what the firmware does; the plug-in keeps all of it.

- **Timing.** The delay line holds 16,368 samples (371 ms). Longer times are
  made by running the line at a lower sample rate, with an anti-alias low-pass
  that follows the rate. That is where the Monomachine's darker, grainier long
  echoes come from. The longest delay is 4 s (Time 127 at 30 BPM).
- **Changing Time.** The read position moves at most 4 samples per 16, so the
  echoes glide: they bend down in pitch by about 5 semitones while the time gets
  longer and up by about 4 while it gets shorter. On a slowed line the rate
  changes first (10 % of the way per 16 samples); the time only moves once the
  line is back at full rate.
- **Knobs.** Parameter changes reach the delay the way the Monomachine's CPU
  sends a knob: every 384 samples the value moves a quarter of the way to where
  you set it. Modulation is added on top, unsmoothed, every 48 samples, as the
  unit's LFOs are. Tempo changes take effect every 48 samples.
- **Filter and feedback.** High-pass, then low-pass, then the output; the same
  signal, times 2 × Feedback, goes back into the line with the input.

## Differences from the hardware

- **Mix replaces the send.** The input goes into the delay at full send: DSND −64
  (a gain of 1.0) normally, or DSND +63 (0.969) with Ping-Pong, since a positive
  DSND is what selects ping-pong on the unit. The hardware has no dry/wet: it
  adds the delay to the track's signal.
- **No glide-in on load.** A new instance starts at its delay time. The unit
  starts at 8,192 samples and glides to the set time.
- **Tempo** comes from the host's transport, held to the unit's 30–300 BPM
  (120 BPM if the host sends none). With Tempo Sync off it is fixed at 120 BPM;
  the unit itself is always synced to its tempo.
- **Feedback is never cut.** The firmware sets it to zero during one phase of
  the track's amp envelope (a fixed-rate fade). The plug-in has no envelope.
- **Other sample rates.** The delay runs at 44.1 kHz, the unit's rate. At other
  rates the plug-in resamples in and out (windowed sinc), so timing is the same
  in milliseconds but the output is no longer bit-exact. The dry signal is
  never resampled.
- **Levels.** 0 dBFS in the plug-in is full scale at the delay's input, where
  the firmware's feedback path saturates.

Latency is 16 samples at 44.1 kHz (one 16-sample block), 107 at 48 kHz and
213 at 96 kHz. It is reported to the host, and the dry signal is delayed to
match.

## Accuracy

The reference is Monomachine OS 1.32B running in the DSP56300 emulator of
[Monomodule](https://github.com/shnolk/monomodule), with the track parameters
built by Monomodule's model of the Monomachine's CPU. It has not been compared
with a physical Monomachine.

| Check | Result |
|-------|--------|
| Core vs. firmware: random times, feedback, filter, ping-pong, tempo 30–300 BPM, inputs up to full scale, 160,000 blocks (144,000 on a slowed line) | every output sample identical; the time, rate and write position identical after every block |
| Built plug-in code vs. firmware: knob moves through the CPU model, tempo changes and ping-pong switches (2,839 moves in 120,000 blocks) | every output sample identical |
| Tables vs. firmware memory | identical. The formulas match except 15 entries that are one LSB off; those are corrected in code. |

`tests/test_core.c` pins this without the firmware. Its expected hashes were
produced by feeding the same stimulus into the firmware's delay, which the
core matched.

## Build and install

```bash
./build_macos.sh
```

```bash
cp -R "build/Mono Delay.clap" ~/Library/Audio/Plug-Ins/CLAP/
```

The build script needs Xcode's command-line tools.

## Tests

```bash
cc -std=c99 -O2 -Wno-unused-function tests/test_core.c -lm -o tests/test_core && tests/test_core
```

```bash
xcrun clang -std=c11 -O2 -Wno-unused-function -Wno-missing-field-initializers -I vendor/clap/include tests/clap_test.c -o tests/clap_test && tests/clap_test "build/Mono Delay.clap"
```

`test_core` checks the tables and 12,000 blocks of output against the
firmware-derived hashes. `clap_test` loads the built plug-in and checks:

- at 44.1 kHz, output equal to the core's;
- the dry path untouched at 44.1, 48 and 96 kHz;
- the echo time at 48 kHz;
- ping-pong;
- Tempo Sync off;
- state (including a 0.1.0 state) and the parameter text.

The plug-in also passes clap-trap's `validate` (at 44.1, 48 and 96 kHz) and
`state --roundtrip`.

## Source

- `source/mono_delay.h`: the delay core (C99, no dependencies but libm).
- `source/mono_delay_clap.c`: the CLAP plug-in around it.
- `vendor/clap`: CLAP 1.2 headers (MIT).

## License

GPL-3.0-only. The DSP is an independent reimplementation of the Monomachine's
delay algorithm, derived from analysis of Elektron's firmware; no firmware is
included. Monomachine and Elektron are trademarks of Elektron Music Machines;
this project is not affiliated with Elektron.
