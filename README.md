# Demo: Stable Nano-UAV Centering in Confined Spaces using Multi-Zone ToF Sensors

This repository contains the implementation of the MobiCom 2026 demo
**"Stable Nano-UAV Centering in Confined Spaces using Multi-Zone ToF Sensors"**
([poster](docs/mobicom26_tanaka_poster.pdf)).

A Crazyflie 2.1 Brushless equipped with a multi-zone ToF deck (VL53L8CX × 11)
takes off inside a 50 × 50 × 50 cm chamber, hovers at 25 cm, and lands. During
the flight it keeps itself centered with velocity commands computed from the
measured wall distances. While hovering, it also corrects its heading from the
normal of the fitted wall plane.

This implementation is built on top of Bitcraze's
[crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware).

## Environment Setup

This repository uses the build system and toolchain of the original
crazyflie-firmware. To set up the environment (ARM toolchain, Python tools
such as `cfloader`, Crazyradio drivers/permissions, etc.), see the original repository:

- [crazyflie-firmware (GitHub)](https://github.com/bitcraze/crazyflie-firmware)
- [Building and flashing instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md)
- [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)

## Demo Applications

The demo applications are in [`examples/demos/`](examples/demos/):

| Directory | Mode | Description |
|---|---|---|
| [`app_cf_brushless_chamber_takeoff_Pcontrol_hybrid`](examples/demos/app_cf_brushless_chamber_takeoff_Pcontrol_hybrid) | **Proposed**: wall-referenced ToF control | Wall-distance-based centering (P control) plus yaw correction from plane fitting during hover |
| [`app_cf_brushless_chamber_without_Pcontrol`](examples/demos/app_cf_brushless_chamber_without_Pcontrol) | **Baseline**: optical-flow position hold | Holds the takeoff x/y using EKF estimates (wall distances are logged only) |

## Build and Flash

Go to the folder of the demo you want to run, build it, and flash the
firmware to the Crazyflie. The example below uses the proposed method:

```bash
cd examples/demos/app_cf_brushless_chamber_takeoff_Pcontrol_hybrid

# Select the Crazyflie 2.1 Brushless platform (needed only on the first build)
make cf21bl_defconfig

# Build
make -j$(nproc)

# Flash over Crazyradio (put the Crazyflie into bootloader mode first)
make cload
```

To run the baseline, use `cd examples/demos/app_cf_brushless_chamber_without_Pcontrol`
and run the same commands.

> If you previously built for a different platform (e.g. a `build/.config` for CF2),
> run `rm -rf build` before running `make cf21bl_defconfig` again.

## Running the Demo

1. (Optional, for logging) Copy `sdcard/config.txt` from the demo folder to the
   root of the micro SD card, and insert the card into the Crazyflie.
2. Place the Crazyflie at the center of the floor of the 50 × 50 × 50 cm chamber
   and power it on.
3. Start the mission in either of the following ways:
   - Hold your hand over the upward-facing ToF sensor (within 300 mm by
     default, for 500 ms by default).
   - Set the parameter `chamber.start` to `1` (e.g. with cfclient).
4. The Crazyflie arms, takes off to 25 cm, hovers for 10 s (`chamber.hoverTimeS`
   by default), and lands at the same spot.

## License

Like the original crazyflie-firmware, this code is licensed under LGPL-3.0
(see [LICENSE.txt](LICENSE.txt)).
