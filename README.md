> [!NOTE]
> **Index of Spotlight-related repositories:** see [go.epfl.ch/spotlight-poseforge](http://go.epfl.ch/spotlight-poseforge#code).

# spotlight-control

Control software for the Spotlight experimental setup, written in C++ with performance optimizations and multi-threading, plus Arduino firmware for the trigger controller.

## Binaries (built from `recorder/`)

| Binary | Description |
|---|---|
| `run-spotlight` | Main recording GUI. Requires `-p PROFILE_DIR -a ARENA_DIR`. |
| `run-arena-registration-scan` | Automated scan to collect AprilTag images for registration fitting. |
| `align-cameras` | Live preview for physically aligning the two cameras. |
| `pco-camera-server` | Out-of-process server streaming PCO muscle-camera frames via shared memory. |

## Quick start

```bash
# Build
cd recorder/build/
cmake ..
make -j16
make install   # installs to recorder/bin/

# Run
./run-spotlight -p ~/Spotlight/profiles/sibo_260514 -a ~/Spotlight/arenas/arena146
```

See the [Wiki](https://github.com/NeLy-EPFL/spotlight-control/wiki) for the full setup and calibration procedure.
