# RTCBenchmark
RTC benchmarking code for measuring clock drift

# Config
The sdkconfig file specifies the XTAL 32 kHz crystal as the RTC to be used. The calibration phase duration was changed from 1024 cycles to 8192 in hopes of better calibration. Additionally, the number of attempts to repeat calibration was changed from 1 to 10. These value changes are purely arbitrarily chosen and not based on any scientific source of what provides the highest accuracy.

# Benchmark
Current code benchmarks the default internal RC oscillator, comparing its drift to a global clock fetched via SNTP (Simple Network Transfer Protocol). The two clocks are compared at a 10 minute interval over a 10 hour duration, logging the timestamps of the two clocks along with the difference of their offsets to an SD card. This data will thereafter be used for data processing and comparison against other clock configurations.
