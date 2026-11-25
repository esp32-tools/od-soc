# esp32-tools/od-soc

Ultra-lean **On-Device State of Charge** library (`od soc code` + `od soc history`) engineered for the **accuracy-per-resource Pareto knee** on real-time microcontrollers (ESP32, STM32, nRF).

Designed to pair with **[`esp32-tools/ip-soc`](https://github.com/esp32-tools/ip-soc)** in a complementary **2×2 architecture**:

| | **On-Device (`esp32-tools/od-soc`)** | **Remote / IP (`esp32-tools/ip-soc`)** |
| :--- | :--- | :--- |
| **Algorithm** | **`od soc code`** — **455 B** Xtensa code flash, **16 B** RTC RAM, **0 FPU**, ~1.5 µs @ 80 MHz; always on; **sole writer** to on-device flash | **`ip soc code`** — Full 4-state 2RC + Plett hysteresis SR-AEKF + non-causal RTS smoother + cell ID; runs **only while connected over IP**; informs `od soc code` |
| **History** | **`od soc history`** — Always-on, hypercompressed **8 B/event** ring (136 B RTC → 4–8 KB append-only flash); keeps offline accuracy + cold-boot recovery + offline SOH | **`ip soc history`** — High-res 16 B `SOC1` event/telemetry stream offloaded to remote IndexedDB/SSD for free, with flash spillover + ACK-free when disconnected |

## Measured Resource Footprint (`xtensa-esp32-elf-g++ -Os`, ESP-IDF v6.1.1)

| Module | Code Flash (`.text`+`.literal`) | Storage Flash (Persistent Data) | RAM / RTC RAM | Notes |
| :--- | :---: | :---: | :---: | :--- |
| **`od soc code`** (`src/od_soc.cpp`) | **455 B (0.44 KB)** | 0 B | **16 B** (`OdSocState`) | Pure 32-bit integer (0 FPU, 0 `expf`/`sqrtf`, 0 heap, 0 ESP32 FPU core-pinning) |
| **Cold-start + CRC16** (`src/od_soc_util.cpp`) | 910 B *(~250 B linked)* | 52 B (`CellParams` NVS) | 0 B | Validates 52-byte calibration pushed by `ip-soc` |
| **`od soc history`** (`src/od_history.cpp`) | 1,328 B | **4–8 KB** (1–2 sectors = 512–1,024 events) | **136 B** (`RtcEventRing`) | 8 B/event (log-dt 1 s..18.7 d); flushes **only when disarmed** (~1 erase/month) |
| **`SOC1` wire frame** (`src/ip_frame.cpp`) | 534 B *(~260 B encode-only)* | 0 B | 0 B | Compact CRC16-framed `ip soc history` stream records |
| **M5StickC Plus wrapper** (`boards/m5stickc_plus/`) | 900 B (AXP192) + 1,825 B (FuelGauge) | 0 B extra | ~180 B (`RtcFuelGaugeState`) | Contiguous I²C burst reads, 200 Hz active / 25 Hz sleep HW coulomb counter, linear-charger thermal proxy |

## Benchmark vs. Higher-Order Physical Plant (2RC + Plett Hysteresis)

Tested against a strictly harder 2RC + Plett dynamic hysteresis + Arrhenius thermal plant (`sim/plant.hpp`):
- **Dynamic load + Wi-Fi TX bursts (starting from 25% cold-start SOC error):** **`1.54%` RMSE** vs. `6.61%` stateless IR-corrected LUT.
- **8-hour deep-sleep wake catch-up (HW coulomb counter):** **`0.98%` error**.
- **−400 mV I²C glitch injection:** rejected cleanly without corrupting SOC.

## Real-Time & Core-0 Safety Guarantees
- **Pull-driven:** Creates zero FreeRTOS tasks, zero timers, zero mutexes, zero heap allocations.
- **Zero FPU in `od_soc::step()`:** Avoids ESP32 FreeRTOS FPU context switching and automatic FPU core-pinning.
- **Single-flash-writer discipline:** Only `od-soc` writes battery state/history to flash, and only when the caller explicitly invokes `flush_history_if_safe()` / NVS save while disarmed (respecting ESP32 flash-cache-disable safety).

## Quick Host Build & Test

```bash
cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
```

## Shared Binary Wire Contracts (Bit-Exact with `esp32-tools/ip-soc`)
- **`CellParams` (52 bytes LE, CRC-16/CCITT-FALSE):** 17-point uniform OCV table (mV), `q_max` (0.1 mAh), `r0`, `r1` (mΩ), `tau` (s), `v_full` (mV), `i_term` (mA), charging efficiency.
- **`PackedEvent` (8 bytes LE):** 12-bit log-quantized Δt, 4-bit `EventType`, 11-bit mV (`2500–4547`), 11-bit signed mA (`-1024..+1023`), 10-bit signed 0.25 mAh ΔQ, 8-bit 0.5% SOC, 8-bit 0.5 °C temp.
- **`SOC1` frame:** 12-byte header + N × 16-byte `IpRecord` + 2-byte CRC16.
