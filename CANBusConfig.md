### SparkMax Configuration

| Device                       | CAN ID | Smart Current Limit | Fwd Limit | Rev Limit | Alt Encoder Zero |
|------------------------------|--------|---------------------|-----------|-----------|------------------|
| FL Drive (Kraken)            | 1      | 60                  | -         | -         | -                |
| FR Drive (Kraken)            | 2      | 60                  | -         | -         | -                |
| BL Drive (Kraken)            | 3      | 60                  | -         | -         | -                |
| BR Drive (Kraken)            | 4      | 60                  | -         | -         | -                |
| FL Spin (Max)                | 11     | 40                  | -         | -         | -                |
| FR Spin (Max)                | 12     | 40                  | -         | -         | -                |
| BL Spin (Max)                | 13     | 40                  | -         | -         | -                |
| BR Spin (Max)                | 14     | 40                  | -         | -         | -                |
| Flywheel Top (Flex)          | 15     |                     | -         | -         | -                |
| Flywheel Bottom (Flex)       | 16     |                     | -         | -         | -                |
| On Wrist Intake Motor (Flex) | 17     | 40                  | -         | -         | -                |
| Wrist Motor (Max)            | 18     | 40                  | -         | -         | [INS]            |
| Main Fixed Motor (Max)       | 19     | 40                  | -         | -         | -                |
| Selector Fixed Motor (Max)   | 20     | 40                  | -         | -         | -                |
| Climb Left (Flex)            | 23     | 80                  | -         | -         | -                |
| Climb Right (Flex)           | 24     | 80                  | -         | -         | -                |
| Elevator Lift Motor (Kraken) | 25     | 60                  | -         | -         | -                |
| Elevator Amp Motor  (Max)    | 26     | 40                  | -         | -         | -                |
| Flywheel Angling Motor (Flex)| 30     | 40                  | -         | -         | [INS]            |


### RoboRIO Configuration

| Device               | DIN Port |
|----------------------|----------|
| Intake Main Sensor   | 4        |
| FL Enc               | 0        |
| FR Enc               | 1        |
| BL Enc               | 2        |
| BR Enc               | 3        |
