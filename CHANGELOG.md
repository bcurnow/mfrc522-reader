<!-- MDTOC maxdepth:6 firsth1:1 numbering:0 flatten:0 bullets:1 updateOnSave:1 -->

- [1.0.3](#103)
- [1.0.2](#102)   
- [1.0.1](#101)   
- [1.0.0](#100)   

<!-- /MDTOC -->

# 1.0.3
* Fixed a bug in `transceive()` where a valid RFID response arriving on the final countdown iteration was incorrectly reported as `COUNTDOWN_TIMEOUT`.
* Fixed a bug in `transceive()` where `results_len` was calculated before clamping `bytes_written` to `FIFO_BUFFER_MAX_SIZE`, causing it to reflect an unclamped byte count.
* Fixed a bug in `__init__()` where the SPI file descriptor was leaked if construction raised after `spi.open()`; `atexit` handler registration is now skipped in that case too.
* Fixed a bug in `close()` so it is idempotent — a second call (e.g. from `atexit` after an explicit close) no longer double-closes the SPI device.
* Fixed a bug in `read_uid()` where the deadline was checked after sleeping, causing `timeout=0` to sleep an unnecessary 1 ms before returning.
* Fixed a bug in `read_uid()` where only values `<= -1` produced an infinite wait; all negative values now correctly produce an infinite wait as documented.
* Fixed a bug in `anticollision()` where an unexpected cascade level left `uid_start_index` unbound; it now returns `ERR` instead.
* Fixed a bug in `anticollision()` where a PICC signalling more cascade levels after CS3 would raise an unintelligible `ValueError`; it now returns `ERR`.
* Added `--privileged` to `make docker-run` so that `RPi.GPIO` correctly detects the Raspberry Pi hardware inside the container.
* Added `make release` target to automate building and publishing GitHub releases.

# 1.0.2
* Found an off by one error impacting double and triple PICC type A tags. This error resulted in an index out of bounds for triple and an extra, incorrect, zero for a double.

# 1.0.1
* Added additional logic to handle cases where the GPIO mode is already set (e.g. by another library like NeoPixel or another process). All cases are handled gracefully except when the current GPIO mode doesn't match the requested GPIO mode and a custom rst_pin has been provided. This case now throws a ValueError with a message including hints on how to fix this problem (e.g. switch the mode and provide the equivalent rst_pin for the other mode).

# 1.0.0
* Initial release
