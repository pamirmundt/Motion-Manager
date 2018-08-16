# About
#### SD card provides configuration parameters to MotionManager.
#### MotionManager only reads SD card on boot, during logo reveal.
#### MotionManager should be turned off and on, if any changes are done on the SD card.
#### If no SD card is provided or there is a missing .csv file, MotionManager uses its build in default parameters.
#### All parameters are in floating points and the syntax is "10.0,20.0,30.0", with commas between parameters and without any spaces.


## PID.csv

### (Kp,Ki,Kd,integralMin,integralMax,alpha)
#### Kp: Proportional Coefficient
#### Ki: Integral Coefficient
#### Kd: Derivative Coefficient
#### integralMin: Integral Sum Minimum Threshold
#### integralMax: Integral Sum Maximum Threshold
#### alpha: Moving average filter alpha value, [ accumulator = (alpha * new_value) + (1.0 - alpha) * accumulator ]

##### More about PID: https://en.wikipedia.org/wiki/PID_controller
##### More about moving average filter: https://en.wikipedia.org/wiki/Moving_average

## RPM.csv

### (refRpmResolution, refRpmUpperThreshold)
#### refRpmResolution: Reference RPM Resolution
#### refRpmUpperThreshold: Maximum Selectable Reference RPM

## encoder.csv

### (encoderPulsePerRev)
#### encoderPulsePerRev: Encoder Resolution / Pulses Per Revolution

##### More about encoders: https://en.wikipedia.org/wiki/Rotary_encoder

## flowRateVersusVoltage.csv

### (A,B,C)
#### Quadratic function constants for Flow Rate(ml/min) versus voltage
#### Flow Rate = A * (Voltage)^2 + B * voltage + C

##### Quadratic curve fitting: https://www.wolframalpha.com/input/?i=quadratic+fit&lk=3

## voltage.csv

### (voltageResolution,voltageUpperThreshold)
#### voltageResolution: Selectable voltage resolution
#### voltageUpperThreshold: Maximum selectable voltage
