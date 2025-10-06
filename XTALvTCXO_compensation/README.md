# RTCBenchmark
RTC benchmarking code for measuring clock drift. The benchmarking will examine the performance of three different clock configurations: Default RTC, Internal oscillator connected to an external 32 kHz crystal and an external Temperature-Compensating Crystal Oscillator Module.

# Benchmark
Current code benchmarks the default internal RC oscillator, comparing its drift to a global clock fetched via SNTP (Simple Network Transfer Protocol). The two clocks are compared at a 10 minute interval over a 10 hour duration, logging the timestamps of the two clocks along with their offsets to an SD card. This data will thereafter be used for data processing and comparison against other clock configurations.
