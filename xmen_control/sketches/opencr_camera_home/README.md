# Camera pan/tilt commands

XM430-W350, Protocol 2.0, 1 Mbps. Pan ID11, tilt ID12.
Saved home: pan 4067 ticks, tilt 2116 ticks, homing offsets 0.
Angles are relative to the saved home, not relative to each command's starting position.

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200 --eol LF -e
```

| Command | Meaning |
|---|---|
| `pan 30` | ID11 to home +30 degrees; positive was observed to turn left |
| `tilt -10` | ID12 to home -10 degrees; physical sign must be observed |
| `speed 5` | Future moves use up to 5 degrees/s (motor resolution rounds down) |
| `h` | Tilt home first, then pan home |
| `x` | Immediate input handling; stop at current position and retain torque |
| `off` | Release both motors; support the camera before entering |
| `p` | State, position, torque and motor hardware error |
| `auto on` | Save automatic homing on board startup in OpenCR EEPROM |
| `auto off` | Save waiting for commands on startup |

Move completion does not disable torque. Commands other than x are line-based.
Backspace is supported. Send x before changing commands during movement.
Default speed is 5 deg/s. Speed setting range is 1.374..30 deg/s and the motor's configured velocity limit is checked.
Pan targets: -90..90 deg. Tilt targets: -135..135 deg. These are software test ranges, not measured mechanical clearances.
On first activation, current position must lie within the same home range.
After off, the next movement rereads and establishes the home branch.
Return follows the calibrated home branch across the 0/4096 encoder boundary.
Do not rotate the camera multiple full turns manually: the stored single-turn home cannot identify cable winding.

First use: keep camera supported, ensure cable slack throughout the route, enter p, then h.
Only enable auto on after observing a successful complete return. Automatic homing happens on OpenCR firmware startup; restoring only motor power while the board remains running does not itself restart homing.
Communication/hardware faults latch; p reports the fault. Stop and inspect before restarting the board.
If holding fails, firmware requests torque off; camera support may be necessary.

The controller uses extended position mode and motor internal position PID, unlike the original OpenCR P-control experiment. It is a camera positioning tool, not evidence for the external-P-control exercise.
