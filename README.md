# MotionManager
Controller for HNP Mikrosysteme micro annular gear pump MZR-7223
- 100 Hz PI RPM control with a RPM MAP filter
- 5 Hz screen refresh
- 31.250 Hz PWM frequency

## Future Improvements
- Spaghetti code
- SD card reader and LCD need different SPI busses. (because LCD SPI does not have a CS option where SPI messes)
- 24V converter for fixed input
- Converter efficiency and heat problems with linear regulators (Linear regulator must be replaced with a switching converter)
- Voltage protection
- SPI is not enough for LCD, additional data bus lines should be used for feedback from LCD
