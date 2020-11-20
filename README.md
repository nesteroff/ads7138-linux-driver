# TI ADS7138 Linux Driver
This repository contains a driver for TI ADS7138 8-Channel 12-Bit ADC.

# Device Tree Example
```
&i2c1 {
  ...
    
  adc: ads7138@16 {
    compatible = "ti,ads7138";
    reg = <0x16>;
    AVDD-supply = <&reg_5p0v>;
  };
  
  ...
}
```

# Reading Data
```
$ cat /sys/class/hwmon/hwmon1/in0_input
5000
```
Returned value is the voltage in millivolts (mV).
