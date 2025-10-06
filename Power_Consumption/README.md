# Power Consumption Measurement (External 32 kHz Crystal)

This minimal ESP-IDF project enters deep sleep immediately so you can measure baseline current when only the RTC slow domain (with an external 32 kHz crystal) is running.

## Goal
Obtain the lowest realistic deep sleep current attributable to: RTC slow clock + retention of RTC slow memory. All high-power domains are turned off.

## Key Configuration
- External 32 kHz crystal selected via `CONFIG_ESP32_RTC_CLK_SRC_EXT_CRYS` (see `sdkconfig.defaults`).
- No wake source configured by default (device will remain in deep sleep until reset or power cycle).
- Logging level reduced to ERROR to minimize pre-sleep activity.

## Hardware Requirements
- 32.768 kHz crystal properly connected to ESP32 XTAL32K pins (and load capacitors as per reference design).
- Stable power supply and ability to insert an ammeter (e.g. uCurrent, Otii, Joulescope, or DMM in µA range). Keep leads short.
- Remove / cut power to any external LEDs, UART adapters, or other boards that inject current.

## Usage
1. Connect the board for initial flashing only (USB / UART). Keep measurement setup clean; after flashing you can remove the USB 5V if powering from lab supply.
2. From this directory run:

```bash
./run.zsh
```
3. After log shows entering deep sleep, stop the monitor (Ctrl+]).
4. Disconnect the USB (if it back-powers anything) and place meter in series with supply.
5. Press EN/reset to re-enter deep sleep quickly and observe current.

## Optional: Add a GPIO Wake Source
If you need a practical way to wake without resetting, you can enable a GPIO wake:
- Edit `main/power_measure.c` and set `CONFIG_POWER_MEASURE_ENABLE_GPIO_WAKE` to 1 (or create a menuconfig option later).
- Choose a valid RTC IO pin for `CONFIG_POWER_MEASURE_WAKE_GPIO` (e.g., 33). Ensure external pull state matches configured trigger.

Rebuild & flash, then pull that pin low (or high depending on config) to wake. Each wake cycle will re-log then sleep again.

## Expected Current
Exact numbers depend on silicon revision, board leakage, crystal drive, power rail quality. Consult ESP32 datasheet (Deep-sleep w/ ext 32 kHz). Boards with USB‑serial chips or regulators often add quiescent current; consider isolating the module.

## Next Steps
- Create parallel project measuring DS3231 supply current separately.
- Add measurement logging via external meter (SCPI) script, if desired.
- Introduce wake timer (esp_sleep_enable_timer_wakeup) to periodically sanity-check RTC drift.

## License
Internal project documentation.
