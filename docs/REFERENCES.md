# References

## User-Provided Context

- ROVSub: https://rovsub.it/index.php/project/stacys-photo-set-2/

The ROVSub URL was user-provided context. It is retained as provenance for the
BlueROV2 Heavy-style discussion, but it is not used for numeric thrust values if
it is inaccessible.

## Official Vehicle Reference

- Blue Robotics BlueROV2: https://bluerobotics.com/store/rov/bluerov2/

BlueROV2 configuration facts come from official Blue Robotics documentation:

- BlueROV2 is available in six- and eight-thruster configurations.
- BlueROV2 uses T200 thrusters in a vectored configuration.
- The Heavy Configuration Retrofit Kit expands the vehicle to eight thrusters.
- The Heavy configuration provides full six-degree-of-freedom control and
  improved stability.
- This project does not hard-code a real BlueROV2 motor order.

## Official Thruster Reference

- Blue Robotics T200: https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/

T200 thrust, current, and power data come from official Blue Robotics T200
specifications. The model converts thrust with `1 kgf = 9.80665 N`.

| Voltage | Forward thrust | Reverse thrust | Full-throttle current | Full-throttle power |
| --- | ---: | ---: | ---: | ---: |
| 12 V | 3.71 kgf | 2.92 kgf | 17 A | 205 W |
| 16 V nominal | 5.25 kgf | 4.10 kgf | 24 A | 390 W |
| 20 V maximum | 6.70 kgf | 5.05 kgf | 32 A | 645 W |

The official T200 operating-voltage range is 7-20 V. Nominal operation at
12-16 V is recommended for thrust/efficiency balance; operation up to 20 V is
allowable, while exceeding 20 V is not recommended.
