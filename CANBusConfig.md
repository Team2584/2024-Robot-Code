### SparkMax Configuration

| Device                       | CAN ID | Smart Current Limit | Fwd Limit | Rev Limit | Alt Encoder Zero |
|------------------------------|--------|---------------------|-----------|-----------|------------------|
| FL Drive (Kraken)            | 1      |                     | -         | -         | -                |
| FR Drive (Kraken)            | 2      |                     | -         | -         | -                |
| BL Drive (Kraken)            | 3      |                     | -         | -         | -                |
| BR Drive (Kraken)            | 4      |                     | -         | -         | -                |
| FL Spin (Max)                | 11     |                     | -         | -         | -                |
| FR Spin (Max)                | 12     |                     | -         | -         | -                |
| BL Spin (Max)                | 13     |                     | -         | -         | -                |
| BR Spin (Max)                | 14     |                     | -         | -         | -                |
| Flywheel Top (Flex)          | 15     |                     | -         | -         | -                |
| Flywheel Bottom (Flex)       | 16     |                     | -         | -         | -                |
| On Wrist Intake Motor (Flex) | 17     |                     | -         | -         | -                |
| Wrist Motor (Max)            | 18     |                     | -         | -         | [INS]            |
| Main Fixed Motor (Max)       | 19     |                     | -         | -         | -                |
| Selector Fixed Motor (Max)   | 20     |                     | -         | -         | -                |
| Climb Left (Flex)            | 23     |                     | Y         | -         | -                |
| Climb Right (Flex)           | 24     |                     | Y         | -         | -                |
| Elevator Lift Motor (Max)    | 25     |                     | -         | -         | -                |
| Elevator Amp Motor  (Max)    | 26     |                     | Y         | -         | -                |
| Flywheel Angling Motor (Max) | 30     |                     | -         | -         | [INS]            |


### RoboRIO Configuration

| Device               | DIN Port |
|----------------------|----------|
| Intake Main Sensor   | 9        |
| Intake Tunnel Sensor | 8        |
| FL Enc               | 0        |
| FR Enc               | 1        |
| BL Enc               | 2        |
| BR Enc               | 3        |