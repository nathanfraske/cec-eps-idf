# CLAUDE.md — agent working context for `cec-eps-idf`

Everything a future agent needs to work on this firmware with full context.
Read this first. It captures hard-won knowledge (especially ESP-IDF 6.x
gotchas) that is expensive to rediscover.

**Maintain this file every revision**, alongside `README.md` (architecture
spec) and `FOLLOWUPS.md` (deferred-work tracker). If you change behavior,
update the README; if you defer something, log it in FOLLOWUPS; if you learn
something a future agent would waste time rediscovering, put it here.

---

## 1. What this is

Firmware for the **EPS current-monitoring module** of the Critical Error
Computing (CEC) PC power-monitoring platform. It watches current on the two
EPS (CPU power) cables feeding a motherboard, classifies load, detects
anomalies, captures high-rate transients, and reports to a central Hub over
CAN.

It is the **sibling** of [`cec-24pin-idf`](https://github.com/nathanfraske/cec-24pin-idf),
which monitors the 24-pin ATX rails. The two share an architecture, component
layout, naming convention, and several byte-for-byte-identical primitives. The
**parity relationship is a first-class design constraint** — see §11. When you
touch a shared primitive here, the same change is owed to the 24-pin (log it in
`FOLLOWUPS.md` under "24-pin parity").

Prototype hardware uses ACS758 Hall sensors on the analog ADC; production will
move to INA226 + shunt over I²C. Detection / protocol / capture layers carry
over unchanged — only the sensor driver swaps.

## 2. Hardware

- **MCU**: ESP32-S3 N16R8 (Lonely Binary dev board) — 16 MB flash, 8 MB
  **octal** PSRAM. Two USB-C ports (see §9).
- **Current sensors**: 2× ACS758LCB-050B, Hall-effect, bidirectional, ±50 A,
  40 mV/A at 5 V, output centered Vcc/2. Analog output through a 2:3 divider
  (10k/20k) into ADC1.
- **Supply quirk**: the dev board's USB-Vbus rail reads **4.4 V**, not 5 V.
  The ACS758 is ratiometric, so quiescent and sensitivity scale with Vcc. This
  is handled — `CEC_DEFAULT_SUPPLY_V = 4.4f` and `acs758_set_supply()` rescale
  both. At 4.4 V the zero-current point is ~2.2 V at the chip / ~1.467 V at the
  ADC pin / ~1937 raw (not the 1.67 V / 2210 of the nominal-5 V case).
- **NTC**: 10k @ 25 °C, B=3950, 10k series pull-up to 3V3, on GPIO 7.
- **12 V rail tap**: 47k/10k divider (`V_rail = V_pin × 5.7`) on GPIO 1.
- **CAN**: SN65HVD230 transceiver (Waveshare bench breakout) on the
  daughterboard.

### Pin map (authoritative — matches `main/eps_main.c`)

| GPIO | ADC ch | Function |
|---|---|---|
| 6  | ADC1_CH5 | ACS758 #1 current (EPS cable 1) |
| 10 | ADC1_CH9 | ACS758 #2 current (EPS cable 2) |
| 1  | ADC1_CH0 | 12 V rail tap (47k/10k divider) |
| 7  | ADC1_CH6 | NTC thermistor |
| 4  | — | CAN TX (TWAI) |
| 15 | — | CAN RX (TWAI) — **moved from GPIO 5** when the daughterboard landed |
| 43 | — | UART0 TX (telemetry transport, to CH340K USB-C) |
| 44 | — | UART0 RX |
| 8/9 | — | I²C SDA/SCL (reserved, unused on EPS) |

Avoid GPIO 3 (ADC1_CH2) — JTAG strapping pin.

## 3. Build / flash / monitor

- **Toolchain**: ESP-IDF **6.0.1**, target `esp32s3`. (Gotchas in §10 are
  all 6.x-specific — do not assume 5.x APIs.)
- **There is no IDF in the cloud/agent environment.** You cannot compile here.
  Changes are verified by (a) careful static review and (b) the user flashing
  on the bench and reporting back (usually a TelePlot CSV or a serial log).
  Write code defensively and explain what to look for after flashing.
- Commands (run by the user on the bench):
  ```
  idf.py set-target esp32s3      # once
  idf.py build
  idf.py -p <PORT> flash monitor # CLI + logs on the JTAG USB-C port
  ```
- Key sdkconfig (`sdkconfig.defaults`): `CONFIG_SPIRAM_MODE_OCT=y` (N16R8 is
  octal PSRAM — without it PSRAM won't init), `CONFIG_FREERTOS_HZ=1000`,
  `CONFIG_ESP_TASK_WDT_TIMEOUT_S=10`, `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`,
  custom `partitions.csv` (3 MB factory app + 24 KB NVS).
- `-Werror` is on. The compiler is strict (see §10 format-string gotcha).

## 4. Repo layout

```
main/
  eps_main.c     app_main, the three tasks, CLI command handlers, detector glue
  cec_config.{c,h}  NVS-backed config + L3 profile persistence (uses cec_nvs)
components/
  cec_common/    cec_state.h — shared types (state, config, enums, flags, triggers)
  cec_sensors/   cec_adc (ADC1 wrapper), acs758 (current), ntc (thermistor)
  cec_filters/   ema_t + median_t primitives (caller-owned buffers)
  cec_detection/ cec_layer1/2/3, cec_swing, cec_classifier, cec_detection (orchestrator)
  cec_capture/   pre-trigger ring + 10 kHz adc_continuous HS burst + dump
  cec_comms/     cec_can — TWAI via esp_twai node-handle API
  cec_telemetry/ cec_teleplot — TelePlot output + UART transport hand-off
  cec_cli/       line-based serial command dispatcher
```

Naming: shared component sources carry the `cec_` prefix (`cec_capture.c`,
`cec_teleplot.c`, `cec_can.c`); chip-specific drivers keep their part name
(`acs758.c`, `ntc.c`). Types use the `_t` suffix.

## 5. Runtime architecture

Three FreeRTOS tasks (set up at the end of `app_main`):

| Task | Core | Rate | Job |
|---|---|---|---|
| `sample_task` | 0 | 50 Hz | read ADC → filter → detect → update shared state → push capture ring → fire auto-triggers |
| `output_task` | 1 | 10 Hz | snapshot state under mutex → emit TelePlot |
| `comms_task` | 1 | 20 Hz | snapshot state → CAN telemetry (+ anomaly frame if flags) — only created when `CEC_CAN_ENABLED` |

Plus the `cec_cli` reader task and the `cec_capture` dispatcher task (Core 1,
created in `cec_capture_init`).

**Shared state** (`cec_state_t g_state`): `sample_task` is the sole writer;
readers take `g_state.mutex` briefly to snapshot. Don't add other writers.

**Important scheduling lesson (already fixed, don't regress):**
- `output_task` / `comms_task` use `vTaskDelay`, **not** `vTaskDelayUntil`. A
  long burst dump on Core 1 starves them; `vTaskDelayUntil` would then return
  instantly every iteration ("catch-up" spin) and starve IDLE → WDT panic.
- The capture dispatcher drops its own priority to 1 during the dump and
  `vTaskDelay(1)`s every 64 samples so IDLE1 keeps feeding the WDT.

## 6. Detection system

Layered, mirroring the 24-pin. All operate per-cable. `cec_detection_run`
folds results into a `CEC_FLAG_*` bitfield + a `cec_load_state_t`.

- **Layer 1** (`cec_layer1`): static current threshold. WARNING immediate,
  CRITICAL after `crit_required` (3) consecutive samples → `CEC_FLAG_OVERCURRENT`.
  Also a (currently un-armed, see FOLLOWUPS L2) dropout floor → `CEC_FLAG_DROPOUT`.
- **Layer 2** (`cec_layer2`): rate-of-change (dI/dt) on the **raw** stream.
  `|dI/dt| > 1 A/ms` → `CEC_FLAG_SWING`. EPS-specific: the 24-pin uses an
  adaptive-σ transient detector instead, because cable current legitimately
  swings with CPU load — a rate threshold is the right primitive here.
- **Layer 3** (`cec_layer3` = `cec_rail_profile_t`): running mean+std with
  dual adapt rates (fast 0.01 for first 100 samples, then 0.0005), warm after
  1000 samples. `|z| > 4` for `LAYER3_REQUIRED` (3) consecutive samples →
  `CEC_FLAG_ANOMALY`. **Debounce + adapt-coupling**: the profile is *frozen*
  (not updated toward the sample) while a warm sample is over threshold, so a
  sustained transient can't drag the mean toward itself and defeat the debounce
  / cause on-off chatter. (This was a real bug found on the bench — see git
  history of `cec_detection.c`.)
- **Classifier** (`cec_classifier`): maps (max current, max std) → load bucket
  (IDLE/LIGHT/MODERATE/HEAVY/TRANSIENT).
- **`cec_swing`**: windowed mean-deviation primitive, ported for parity but
  **not yet wired** into anything (see FOLLOWUPS — power-swing).

### Auto-triggers (in `sample_task`)

`cec_trigger_for_flags()` maps the flag bitfield to the most-specific
`cec_trigger_t` so the burst envelope reports the real cause:

| Source | → trigger |
|---|---|
| OVERCURRENT / DROPOUT | `CEC_TRIG_STATIC_CRIT` |
| ANOMALY | `CEC_TRIG_ANOMALY` |
| SWING | `CEC_TRIG_CURRENT_SWING` |
| load-state edge | `CEC_TRIG_STATE_CHANGE` |
| bus-voltage rate < −0.5 V/s | `CEC_TRIG_SHUTDOWN` (bypasses cooldown; arms a 30 s mute window) |
| CLI `burst` | `CEC_TRIG_MANUAL` |

Layers are runtime-toggleable (`set layer1 off`, etc.); a disabled layer still
updates its internal state so re-enabling isn't stale.

## 7. Burst capture (the EPS divergence worth understanding)

`cec_capture` keeps a 50 Hz pre-trigger ring (20 s) in PSRAM. On a trigger it:
1. `cec_adc_pause()` — hands ADC1 from the oneshot driver to `adc_continuous`.
2. Runs a **10 kHz/channel** DMA capture for 1 s into a separate PSRAM buffer.
3. `cec_adc_resume()` — re-acquires oneshot, re-applies channel configs.
4. Dumps pre-trigger ring + HS buffer as TelePlot lines.

`adc_continuous` and `adc_oneshot` **cannot coexist on one ADC unit** — hence
the pause/resume hand-off. `sample_task` checks `cec_capture_is_busy()` and
skips ADC work during the ~1 s burst window. This 10 kHz DMA path is the main
EPS-vs-24-pin divergence (the 24-pin uses a 1 kHz oneshot callback).

**Dump format**: two parallel envelopes. Lowercase `>burst_begin:<ts>:<reason_int>`
/ `>burst_end:<ts>:0` carry numeric values so they survive TelePlot's CSV
exporter (which drops non-numeric value fields). Uppercase
`>BURST_BEGIN:<reason>:...` / `>BURST_END` stay for raw-stream + 24-pin tooling.
HS dump decimation (`set decim N`, default 5) thins what's *emitted* without
thinning what's *captured*.

## 8. Config / NVS

`cec_nvs` wraps ESP-IDF NVS with a 4-byte magic prefix per blob — bump the
magic when a payload layout changes and the loader rejects the stale blob
cleanly instead of loading garbage.

| Key | Magic | Payload |
|---|---|---|
| `config` | `0xCEC50002` | `cec_config_t` (id, supply_v, oc, alpha, output_raw, layer1/2/3_enabled) |
| `zero_off0`/`1` | `0xCEC50101` | float per-sensor ACS758 zero offset |
| `l3_profiles` | `0xCEC50201` | `cec_rail_profile_t[2]` — L3 baselines, saved every 5 min, loaded at boot |

When you add a field to `cec_config_t`, **bump `MAGIC_CONFIG`**. NVS namespace
is `"cec"`. First-flash advice for the user: `esptool erase_flash` so stale
blobs don't shadow new defaults.

## 9. Transports (dual USB-C — important)

The board has two USB-C ports; the firmware uses both:
- **JTAG USB-C** (native USB-Serial-JTAG): CLI input, `ESP_LOG`, boot banners.
  This is the IDF console. `idf.py monitor` connects here.
- **UART USB-C** (CH340K bridge → UART0 GPIO 43/44): **all TelePlot output**
  (telemetry + burst dumps). `cec_telemetry_init_uart()` installs the driver
  and `teleplot_*` / `dump_burst` route through `uart_write_bytes`. Falls back
  to stdio if UART init fails (cable unplugged) so nothing breaks.

Baud is **921600** (`EPS_TELEMETRY_UART_BAUD`). Do **not** assume CH340K does
2 Mbps reliably — it produced garbage on this host; 921600 is the safe standard
rate. The UART clock source is pinned `UART_SCLK_APB` (see §10).

## 10. ESP-IDF 6.x gotchas (catalogue — these cost real time)

Every one of these bit us during bring-up. If something "should work" but
doesn't compile or behaves oddly, check here first.

- **Driver components split out of `driver`.** On 6.x many subsystems moved to
  their own `esp_driver_*` components. `REQUIRES driver` no longer pulls them:
  - TWAI → `esp_driver_twai`, header `esp_twai.h` / `esp_twai_onchip.h`
  - UART → `esp_driver_uart`, header `driver/uart.h`
  - USB-Serial-JTAG VFS → `driver/usb_serial_jtag_vfs.h` (was
    `esp_vfs_usb_serial_jtag.h`), fn `usb_serial_jtag_vfs_use_driver()` (was
    `esp_vfs_usb_serial_jtag_use_driver()`).
- **Legacy `driver/twai.h` is deprecated** and emits a `#warning` →
  `-Werror` build fail. Use the `esp_twai` node-handle API
  (`twai_new_node_onchip` / `twai_node_enable` / `twai_frame_t` /
  `twai_node_transmit` + `twai_node_transmit_wait_all_done`).
- **No hardware CAN loopback on ESP32-S3.** `twai_ll_set_mode` only writes LOM
  (listen-only) and STM (self-test/no-ack); the `enable_loopback` flag is
  silently ignored. You cannot read back your own TX in firmware. Verify CAN
  with a scope / USB-CAN dongle / the actual Hub. `can_init(true)` uses
  self-test (NO_ACK) so TX completes without an external ACKer.
- **`adc_continuous` rejects `ADC_BITWIDTH_DEFAULT`.** Set
  `bit_width = SOC_ADC_DIGI_MAX_BITWIDTH` (12 on S3) explicitly, or you get
  `ESP_ERR_INVALID_ARG` / "ADC bitwidth not supported".
- **UART `source_clk`**: pin it to `UART_SCLK_APB`. `UART_SCLK_DEFAULT` can
  resolve to a clock the bootloader left configured, making the baud divider
  land far off and producing garbage at every nominal baud.
- **`-Werror=format` on `uint32_t`.** Xtensa GCC types `uint32_t` as
  `long unsigned int`, so `%u` errors. Use `PRIu32` (from `<inttypes.h>`) for
  `uint32_t`, and `%" PRId64 "` for `int64_t` (not `%lld`). Cast `uint8_t` to
  `(unsigned)` for `%u`.
- **USB-Serial-JTAG line endings.** Default passes input through unmodified, so
  `fgets` blocks forever on `\r`/`\r\n` from Windows terminals. `cec_cli` sets
  `usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR)`.

## 11. The 24-pin parity discipline

Shared, byte-for-byte (modulo license headers): `cec_filters`, `cec_nvs`,
`cec_cli`, `cec_teleplot` primitives, `cec_swing`, `cec_rail_profile_t`, the
capture trigger enum + dump envelope.

Intentionally divergent (don't "fix" toward the 24-pin): Layer 1 (current vs
voltage band), Layer 2 (dI/dt vs adaptive-σ), capture HS path (10 kHz DMA vs
1 kHz oneshot), state enum (`cec_load_state_t` per-cable vs `cec_state_t`
whole-PSU), UART transport (EPS-only), CAN payload.

`cec_state_t` (24-pin, PSU mode) and `cec_load_state_t` (EPS, per-cable) are
deliberately different names so both modules can include each other's headers.

When you change a shared primitive, add the mirror change to `FOLLOWUPS.md`
under "24-pin parity" so it doesn't drift.

## 12. Git / PR workflow

- Develop on a `claude/<descriptor>` branch. Never push to `main` without
  explicit permission.
- Commit messages: descriptive body explaining the *why*, not just the *what*.
  Do **not** put the model identifier in commits/PRs/code.
- One logical change per commit. Push with `git push -u origin <branch>`.
- PRs only when asked. When subscribed to PR activity, triage events: fix
  small/clear things and push; ask on ambiguous/architectural ones; skip
  no-ops. Reply on the PR only when genuinely useful.
- Past PRs: #1 (initial finalization + parity refactor + CAN/UART bring-up),
  #2 (detection maturation: per-source triggers, L3 z-score + debounce,
  shutdown/state-change triggers, NVS persistence, CSV burst markers).

## 13. How verification works here

You can't build or flash. The loop is: make the change → explain precisely what
the user should see after flashing (boot log lines, TelePlot series, `show` /
`can` CLI output, expected `>burst_begin:<ts>:<reason>` values) → user flashes
and pastes a CSV or serial log → you analyze. Lean on `awk`/`grep` over
uploaded CSVs to quantify behavior. Be honest about what a given test does and
doesn't prove.

## 14. Quick orientation for common tasks

- **Add a detection source**: implement in a `cec_layerN` or new detector, OR
  it into `flags` in `cec_detection_run`, add a `cec_trigger_t` mapping in
  `cec_trigger_for_flags`, fire it in `sample_task`.
- **Add a CLI command**: handler `static int cmd_x(int,char**)` in
  `eps_main.c`, add to `CLI_COMMANDS[]`.
- **Add a persisted config field**: add to `cec_config_t`, set default in
  `cec_config_defaults`, **bump `MAGIC_CONFIG`**, expose via `set`/`show`.
- **Add a telemetry series**: emit in `teleplot_emit_state`; for burst context
  add to `cec_capture_sample_t` + the `dump_burst` pre-trigger line.
- **Change CAN**: `cec_comms/cec_can.c`. Remember the production-flip TODOs.
