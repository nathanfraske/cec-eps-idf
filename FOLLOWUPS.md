# Follow-ups

Tracked deferred work for `cec-eps-idf`. Nothing here is a flash blocker — the
firmware runs live on prototype hardware (current + bus voltage + temp
telemetry, 10 kHz burst capture, CAN in bench loopback). These are the "known
about it, not now" items, kept out of the code and PR descriptions so they
don't get lost.

Maintained per-revision alongside `README.md` and `CLAUDE.md` — when you defer
something, add it here; when you do it, delete the line.

---

## Production flips (grep-loud TODOs in source)

These are deliberate bench-vs-production switches. Each has a banner-style
`TODO` comment in the source so `grep -rn "TODO" main components` finds them.

- **CAN bitrate** — `cec_comms/cec_can.c` `CAN_BITRATE_BPS` is parked at
  `125000` to survive the Waveshare SN65HVD230 breakout's slope-control mode.
  Raise to `500000` (the 24-pin + Hub spec) once on a high-speed transceiver
  or after bridging the breakout's `Rs` to GND.
- **CAN mode** — `main/eps_main.c` calls `can_init(true)` (self-test / NO_ACK).
  Flip to `can_init(false)` for normal mode once the Hub is on the bus and
  ACKing.
- **Hardware: Waveshare `Rs`** — the bench transceiver breakout ties `Rs` to
  GND through 10 kΩ (slope-control). Bridge it straight to GND for high-speed
  mode if you want 500 kbps on the bench instead of the production board.

## Code-review cleanup (lint)

Findings from the PR #2 bench-test review and review passes. Ranked by how much
they'd actually bite. None are active crashes.

| # | Severity | Where | Item |
|---|---|---|---|
| L1 | medium (correctness) | `cec_detection/cec_layer1.c` | OC check is `current >= crit_threshold`, signed. A real **reverse** current (wired cable, PSU sourcing backward) reads negative and silently bypasses overcurrent. Decide the policy: abs-value the current for the OC test, or document that EPS cables are strictly source→load so negative is always a wiring fault and should flag separately. |
| L2 | medium (correctness) | `cec_detection/cec_layer1.c`, `cec_detection.c` | `CEC_FLAG_DROPOUT` is effectively dead: `dropout_enabled` is initialised false in `cec_layer1_init` and never armed anywhere. Wire it to a "system should be loaded" signal (e.g. arm once bus voltage is up and load_state has been ≥ LIGHT) or remove the flag until there's a real arming condition. |
| L3 | medium (data quality) | `cec_detection.c`, `main/eps_main.c` | An **unpopulated** ACS758 channel floats and reads a large constant (≈ −47 A on the bench with EPS2 unwired). That pollutes `max(std_dev)` into the classifier and keeps L3's profile updating on garbage for that cable. Add a `cable_enabled[CEC_NUM_CABLES]` config knob (persisted) so unpopulated channels are excluded from detection, the classifier, and CAN payload. |
| L4 | low (nit, units) | `main/eps_main.c` `bus_shutdown_check` | `slope_v_per_s = bus_v - oldest` is a raw ΔV over the history window, compared against a `-0.5f` "V/s" threshold — correct **only because** `BUS_V_HIST_SIZE` = `SAMPLE_RATE_HZ` makes the window exactly 1 s. Changing either constant silently breaks the units. Divide by the real window seconds, or add a `_Static_assert` tying the two together. (Same class of bug as the 24-pin's L6.) |
| L5 | low (smell) | `main/eps_main.c` L3 save block | `cec_config_save_l3_profiles` (→ `nvs_set_blob` + `nvs_commit`) runs synchronously in the 50 Hz sample task every 5 min. Flash erase/write disables cache on both cores — a multi-hundred-ms loop stall when it fires. Move the periodic save to a low-priority one-shot task. (Same as the 24-pin's L3.) |
| L6 | low (doc) | `cec_detection/cec_layer2.c` | Layer 2 (dI/dt) only sees the 50 Hz sample feed, so it's blind to sub-20 ms transients that the 10 kHz HS path would catch. Either document this clearly as "L2 is the coarse fast-path; HS burst is the fine one," or add a streaming-L2 path that runs on HS samples during capture. |

## Deferred features

- **Per-load-state L3 profiles.** EPS keeps one `cec_rail_profile_t` per cable.
  A sustained shift to a new operating level keeps `CEC_FLAG_ANOMALY` asserted
  forever (the freeze-during-anomaly logic never relearns it as normal). The
  24-pin solves this with per-`(state, rail)` profile banks. Port that pattern
  so a genuine new steady-state gets relearned after a dwell. Until then, burst
  cooldown throttles the captures and the load classifier reports the bucket.
- **CAN payload: bus voltage.** The 8-byte telemetry frame is full
  (type, id, 2×int16 current, flags, temp). Now that we measure the 12 V rail,
  the Hub protocol should grow a voltage field — needs Hub-side coordination on
  the new layout. Deferred to a cross-module protocol revision.
- **Power-swing detector.** We now have bus voltage and per-cable current, so a
  windowed swing detector on `V × I` (the 24-pin's `cec_swing` on power) is
  basically free to wire in. `cec_swing.{c,h}` is already ported as a primitive
  but isn't fed by anything yet. Decide whether power-swing should trigger
  bursts or just be tracked telemetry before wiring.
- **Per-cable voltage taps.** Single divider on the shared rail today. Two taps
  (one per cable) would resolve voltage drop per cable — a connector-resistance
  / cable-degradation diagnostic. Deferred until a use case needs it.
- **Bus-voltage trim.** `s_rail_12v.trim` is unity. Add a `set vtrim <f>` CLI
  command (+ NVS persistence) so the 47k/10k divider can be calibrated against
  a known reference, the same way `cal`/`set supply` calibrate the current path.
- **`set cooldown <ms>`.** Burst cooldown is the compile-time
  `EPS_BURST_COOLDOWN_MS` (10 s). A runtime knob would help bench iteration
  without rebuilding. Low effort.
- **CAN bus-off recovery for production.** In normal mode (`can_init(false)`),
  bus-off becomes a *real* signal (Hub not ACKing, bad termination). The
  `on_state_change` auto-recover is wired, but production may want a backoff /
  alert path rather than silent re-recovery. Revisit when CAN goes normal-mode.

## 24-pin parity (cross-repo TODO)

Items the EPS side already does that the 24-pin (`cec-24pin-idf`) should adopt
for the two trees to stay in lock-step. Carried in the EPS README's parity
section too; this is the actionable checklist.

- **Extract `cec_nvs`** into its own component on the 24-pin (currently inside
  `cec_detection`). EPS has it standalone.
- **Extract `cec_filters`** into its own component on the 24-pin (currently
  inside `cec_detection`). EPS has it standalone.
- **Add `cec_common`** on the 24-pin: move `cec_state.h` out of `cec_detection`
  so the shared types live in one place.
- **Shared enums** — add `cec_load_state_t` + `CEC_LOAD_COUNT` and
  `cec_severity_t` to the 24-pin's `cec_state.h` (markers already noted there)
  so both repos can include each other's headers without clashes.
- **`cec_cli` IDF 6.x headers** — the 24-pin's `cec_cli.c` still includes the
  pre-6.x `esp_vfs_usb_serial_jtag.h` + `esp_vfs_dev.h` and calls
  `esp_vfs_usb_serial_jtag_use_driver()`. On IDF 6.x these are
  `driver/usb_serial_jtag_vfs.h` + `usb_serial_jtag_vfs_use_driver()`. Will
  fail to compile the moment the 24-pin builds on 6.x. (EPS already migrated.)
- **`cec_comms` / `esp_twai`** when CAN ships on the 24-pin: use the
  `esp_twai_onchip` node-handle API directly (skip deprecated `driver/twai.h`).
  EPS `cec_comms/cec_can.c` is the reference; adapt frame layout / IDs.
- **UART transport** — if the 24-pin's USB Serial-JTAG throughput becomes the
  bottleneck, EPS's `cec_telemetry_init_uart` hybrid (CLI on JTAG, TelePlot on
  a CH340-class UART) is the reference pattern.
- **ACS712 driver style** (optional) — the 24-pin's const-config + measure-
  helper shape vs EPS's runtime-mutable cal ctx. Moot once the 24-pin rails
  move to INA226; only relevant if either driver is rewritten.

## Documentation note (TelePlot ms label)

`cec_capture` HS rows use `ts_us_offset` (microseconds since capture start) in
the value-timestamp slot. TelePlot labels its time column `timestamp(ms)`
regardless, so HS plots read "ms" when they're really µs. Not a bug — a label
quirk worth a one-liner wherever capture CSVs get analysed.
