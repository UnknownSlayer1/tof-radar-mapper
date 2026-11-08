# 360 Degree ToF Radar Mapper

An ESP32 based rotating time of flight radar that sweeps a VL53L0X laser
ranging sensor through a full 360 degrees on a stepper motor and builds a
2D occupancy map of the room. The sensor spins, takes 128 range readings
per revolution, and the firmware turns them into a top down map you can
read in the Serial Monitor or plot on a PC.

## Why this matters for a CompE portfolio

This is a miniature version of the sensing stack inside robot vacuums,
warehouse robots, and defense lidar systems. The same ideas show up
everywhere autonomous systems need to know what is around them:

* **Sensor systems:** driving a time of flight sensor over I2C at the
  register level, handling its timing budget, and telling a real target
  apart from a "no target" status.
* **Embedded mapping:** a log odds occupancy grid with ray casting, the
  standard representation for robot mapping, running in fixed memory on
  a microcontroller.
* **Mechatronics:** closed loop angle tracking on a geared stepper,
  backlash aware homing, and the tradeoff between angular resolution
  and scan time.
* **Testable firmware:** all mapping logic is pure C++ with no Arduino
  dependency, so it compiles and runs under g++ on a PC with 53 host
  tests. The Arduino sketch is thin glue only.

## Parts list

Prices are estimates from AliExpress single unit listings checked
Nov 8, 2026. Verify at checkout. Prices are before shipping, and
shipping is often free on these parts but varies by seller.

| Part | Est. price | Search term on AliExpress |
| --- | --- | --- |
| ESP32 DevKit V1 clone | ~$4.00 | ESP32 DevKit V1 |
| VL53L0X GY-VL53L0XV2 time of flight module | ~$1.00 | VL53L0X GY-VL53L0XV2 |
| 28BYJ-48 5V stepper motor + ULN2003 driver board | ~$1.20 | 28BYJ-48 ULN2003 |
| Rotating platform: lazy Susan bearing + acrylic disc, or a 3D printed disc | ~$3.00 | lazy Susan bearing 3 inch / acrylic disc |
| 2x 18650 battery holder + cells, or a USB power bank | ~$4.00 | 18650 battery holder 2 slot |
| Breadboard, jumper wires, micro USB cable | ~$2.00 | breadboard jumper wire kit |
| Standoffs, screws, small breadboard mount | ~$2.00 | M3 standoff kit |
| **Total** | **~$17.20** | |

Total math: 4.00 + 1.00 + 1.20 + 3.00 + 4.00 + 2.00 + 2.00 = **$17.20**,
so the build lands around $17 to $20 depending on shipping and what you
already own. No specific listing URLs are given here because AliExpress
listings rotate constantly; search the terms above and pick a seller with
high order counts and good ratings.

## Wiring

| From | To | Notes |
| --- | --- | --- |
| VL53L0X VIN | ESP32 3V3 | Module has its own regulator, 3V3 is correct |
| VL53L0X GND | ESP32 GND | Common ground with everything |
| VL53L0X SDA | ESP32 GPIO 21 | Default I2C data |
| VL53L0X SCL | ESP32 GPIO 22 | Default I2C clock |
| ULN2003 IN1 | ESP32 GPIO 16 | Coil drive |
| ULN2003 IN2 | ESP32 GPIO 17 | Coil drive |
| ULN2003 IN3 | ESP32 GPIO 18 | Coil drive |
| ULN2003 IN4 | ESP32 GPIO 19 | Coil drive |
| ULN2003 VCC / + | USB 5V rail | Stepper needs 5V, do not power it from 3V3 |
| ULN2003 GND | ESP32 GND | Common ground |
| 28BYJ-48 | ULN2003 board connector | Plugs straight in, 5 wire connector |

Power notes: the ESP32 plus stepper draw more than a laptop USB port
likes when the motor stalls, so use a powered USB hub or a USB power
bank for the build. If you use 18650 cells, power the ESP32 through its
5V pin from a boost board, never feed raw cell voltage into 3V3.

## Build instructions

Hands on steps. Budget about 2 hours for the first build.

1. **Mount the stepper.** Screw the 28BYJ-48 to the base plate with
   M3 screws, shaft pointing up. The motor body has two mounting ears
   for this.
2. **Fit the rotating disc.** Press or glue the acrylic disc or 3D
   printed disc onto the motor shaft. If you use a lazy Susan bearing,
   bolt the bearing to the base and the disc to the bearing, then couple
   the disc to the motor shaft with a small printed coupler. The disc
   must spin freely with no wobble.
3. **Mount the sensor.** Fix the VL53L0X module at the edge of the
   disc, lens facing outward, with standoffs. The sensor must sit above
   the disc surface so the disc edge never blocks its view.
4. **Mark the home position.** Draw a line on the disc edge and a
   matching mark on the base. This is the zero degree reference the
   firmware homes to.
5. **Wire the sensor.** Connect VIN, GND, SDA to GPIO 21, SCL to
   GPIO 22. Keep the I2C wires short, under 20 cm, to avoid signal
   problems.
6. **Wire the driver.** Connect ULN2003 IN1 to IN4 to GPIO 16 to 19.
   Power the driver board from the 5V rail and connect its ground to
   ESP32 ground. Plug the stepper into the driver board.
7. **Deal with the spinning wires.** The sensor wires twist as the disc
   rotates. Leave a loose service loop of wire above the disc, or use a
   slip ring if you want continuous rotation. For this build, the
   firmware sweeps 360 degrees and the wires wind up one turn per scan,
   which is fine for a few scans. Unwind between sessions.
8. **Flash the firmware.** Install PlatformIO, open this project, run
   `pio run -t upload`. Open the Serial Monitor at 115200 baud.
9. **Home the platform.** Line up the disc mark with the base mark,
   then press the BOOT button on the ESP32. The firmware zeroes its
   angle there.
10. **Watch the map.** The first sweep takes about 40 seconds. The
    ASCII map prints when the sweep finishes, then the scan repeats.

## Reading the map output

After each sweep the Serial Monitor prints three blocks.

**ASCII map.** North is up, the robot is `R` at the centre.

```
# occupied cell (something solid here)
. known free space (the beam passed through)
  blank, never observed (behind a wall, or out of range)
R robot position
```

A good scan shows a ring of `#` cells tracing your walls, `.` filling
the room interior, and blank space outside the walls. If the map is
mostly blank, the sensor is not seeing targets: check the wiring and
make sure nothing blocks the lens.

**JSON grid.** `{"cell_mm":50,"cells":81,"robot":[40,40],"grid":[[...]]}`
with one probability per cell, 0.5 meaning unknown. Paste it into any
plotting script to get a heatmap view of the room.

**JSON scan.** One `{"a": angle, "r": range, "hit": bool}` entry per
sample, 128 per revolution. Useful for checking raw sensor behaviour
before the grid smooths it.

**Scan timing.** At 10 ms per half step and 32 steps per sample, one
revolution takes about 41 seconds. To scan faster, lower the step delay
or raise steps per sample, but know the trade: fewer steps per sample
means coarser angular resolution, and a step delay under about 5 ms
makes the 28BYJ-48 skip steps.

## How to run the host tests

The mapping core has no Arduino dependency, so it compiles on a PC:

```bash
./tests/run_tests.sh
```

This compiles `tests/test_main.cpp` with `g++ -std=c++17 -Wall -Wextra`
and runs 53 checks covering polar conversion, grid updates, scan
assembly, out of range handling, stepper wrap, ASCII rendering, and
the VL53L0X driver against a mock I2C transport. Expected output ends
with `53 passed, 0 failed`.

## Project layout

```
platformio.ini          ESP32 DevKit, Arduino framework
src/main.cpp            Arduino sketch, thin glue only
src/polar.h             Polar to Cartesian, angle normalization
src/occupancy_grid.h    Log odds grid, Bresenham ray casting
src/stepper_controller.h 28BYJ-48 sweep control, homing, backlash notes
src/scan_assembler.h    360 deg scan assembly, range filtering, median
src/map_serializer.h    JSON grid/scan output, ASCII map
src/vl53l0x_driver.h    Register level VL53L0X interface + I2C transport
tests/test_main.cpp     Host test suite
tests/run_tests.sh      Build and run the host tests
tests/arduino_shim.h    Minimal Arduino API shim for host builds
```

## Walkthrough

Key concepts to be able to explain out loud, in your own words:

1. **The data path.** The VL53L0X measures one distance per step
   position. The stepper controller turns a step count into an angle.
   The scan assembler pairs angle with range into polar samples. The
   occupancy grid converts each sample to Cartesian and ray casts it
   into log odds cells. The serializer prints the result.
2. **Why log odds, not raw probabilities.** Probabilities saturate at
   0 and 1, so one confident observation can never be undone. Log odds
   add, so evidence accumulates and reverses symmetrically. A cell hit
   once needs about four free passes to clear, which is the right
   balance for a noisy sensor.
3. **Why ray casting matters.** Marking only the endpoint would leave
   the space between robot and wall unknown. Ray casting marks every
   cell the beam crossed as free, which is what turns 128 one dimensional
   readings into a usable 2D map.
4. **The stepper tradeoff.** 4096 half steps per revolution gives
   0.088 deg resolution, but at 10 ms per step a full scan takes 41
   seconds. Resolution, speed, and torque are all traded against each
   other through the step delay and steps per sample.
5. **What the map cannot see.** Glass, mirrors, and black surfaces
   confuse time of flight sensors. A wall the beam hits at a shallow
   angle reflects away instead of back, so the sensor reports no target
   and the map shows free space where a wall is. That is a sensor
   physics limit, not a code bug.

## Mock interview Q&A

**1. Time of flight vs ultrasonic: why did you pick laser ranging?**
Time of flight measures light travel time, so it is fast (about 30 ms
per reading), narrow beam (about 25 degrees, effectively a point), and
accurate to a few millimetres out to 2 metres. Ultrasonic is cheaper
and sees a wider cone, but its beam is 30 plus degrees wide, it is slow
because sound is slow, and soft surfaces absorb it. For mapping, a
narrow beam matters more than a wide one: a wide beam cannot tell you
where along its arc the obstacle is. The tradeoff is that time of flight
fails on glass and mirrors, where ultrasonic would still return an echo.

**2. Why a log odds occupancy grid instead of a plain probability grid?**
Updating a probability directly with Bayes rule needs multiplication
and quickly saturates at 0 or 1, after which new evidence has no effect.
Log odds turn the update into addition, which is cheap on a
microcontroller and never saturates if you clamp it. It also makes the
confidence symmetric: a hit and a miss move the belief by comparable
amounts, so the map can both learn and unlearn as the sensor rescans.

**3. How do you handle noisy or out of range readings?**
Three layers. First, the VL53L0X reports a range status of 0x0B for
no target, which the driver maps to an invalid sample instead of a
zero range, so phantom walls at 0 mm never enter the map. Second, the
scan assembler drops readings below 30 mm or above 2000 mm, the sensor
physical limits. Third, a median of the last three readings at the same
angle kills single sample spikes from multipath or specular glints.
A miss is still used: the ray is cleared out to max range.

**4. Stepper resolution vs scan time: how did you pick the numbers?**
The 28BYJ-48 does 4096 half steps per revolution, 0.088 deg per step.
Taking a reading every 32 steps gives 2.8 deg spacing and 128 samples
per revolution, which is enough to resolve a doorway at 2 metres
(about 10 cm arc per sample). At 10 ms per step the scan takes about
41 seconds. Halving the step delay would halve scan time but the motor
starts skipping steps below about 5 ms, and skipping steps corrupts
every angle after the skip because angle is tracked by counting steps.
Open loop step counting is the weak point, and homing to a physical
mark is what bounds the error.

**5. I2C addressing: what happens if you want a second sensor?**
The VL53L0X default address is 0x29 and the GY-VL53L0XV2 module has no
address select pins, so two sensors on one bus collide. The options are
an I2C multiplexer like the TCA9548A, or one XSHUT line per sensor:
hold all sensors in reset, bring one up at a time, and write a new
address into its register before bringing up the next. XSHUT is
cheaper and uses two GPIO pins per extra sensor.

**6. Why does the map show free space where you know a wall is?**
Two likely causes. One, the beam hit the wall at a shallow angle and
reflected away, so the sensor reported no target and the ray was
cleared as free. Two, the wall is glass or a mirror, which the 940 nm
laser passes through or reflects away from. Both are sensor physics,
not mapping bugs. The fix in a real system is sensor fusion: a second
modality, like ultrasonic or a bump sensor, covering the cases the
laser misses.

**7. How much memory does the grid use, and could it be bigger?**
81 by 81 cells at 4 bytes per float is about 26 KB, which fits easily
in ESP32 RAM. Doubling resolution to 25 mm cells over the same 4 metre
room would be 161 by 161 floats, about 104 KB, still feasible but
leaving less headroom for the rest of the firmware. The real limit is
not RAM but angular resolution: at 2.8 deg spacing, samples at 2 metres
are 10 cm apart, so finer cells would mostly be empty anyway.
