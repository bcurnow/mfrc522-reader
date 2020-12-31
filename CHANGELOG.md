<!-- MDTOC maxdepth:6 firsth1:1 numbering:0 flatten:0 bullets:1 updateOnSave:1 -->

- [1.0.1](#101)   
- [1.0.0](#100)   

<!-- /MDTOC -->

# 1.0.1
* Added additional logic to handle cases where the GPIO mode is already set (e.g. by another library like NeoPixel or another process). All cases are handled gracefully except when the current GPIO mode doesn't match the requested GPIO mode and a custom rst_pin has been provided. This case now throws a ValueError with a message including hints on how to fix this problem (e.g. switch the mode and provide the equivalent rst_pin for the other mode).

# 1.0.0
* Initial release
