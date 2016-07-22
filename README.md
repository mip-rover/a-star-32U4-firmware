# a-star-32U4-firmware
MIP-Rover Firmware for Pololu A-Star 32U4 Robot Controller 

## Register Map

| Name               | Type      | Address |
| ------------------ | --------- | ------- |
| button_a           | UINT8     |       0 |
| button_b           | UINT8     |       1 |
| button_c           | UINT8     |       2 |
| cell_count         | UINT8     |       3 |
| battery_millivolts | UINT16    |       4 |
| low_voltage_cutoff | UINT16    |       6 |
| left_motor_count   | UINT32    |       8 |
| right_motor_count  | UINT32    |      12 |
| left_motor_speed   | UINT16    |      16 |
| right_motor_speed  | UINT16    |      18 |
| left_motor         | INT16     |      20 |
| right_motor        | INT16     |      22 |
| clear_motor_counts | UINT8     |      24 |
| yellow             | UINT8     |      25 |
| green              | UINT8     |      26 |
| red                | UINT8     |      27 |
| play_notes         | UINT8     |      28 |
| notes              | UINT8[16] |      29 |

