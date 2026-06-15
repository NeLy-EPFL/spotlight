# Unit testing

## Host-to-trigger-controller communication protocol

```bash
cd spotlight-control

# Generate Makefile using CMake
cmake -S comm_protocol -B comm_protocol/build

# Run Makefile
cmake --build comm_protocol/build -j 16

# Run unit tests
ctest --test-dir comm_protocol/build --output-on-failure

cd ..
```

## Recorder

To run hardware-independent unit tests,

```bash
ctest --test-dir recorder/build --label-regex nohardware --output-on-failure

# ... or simply (nohardware tests are the only ones registered by default):
ctest --test-dir recorder/build --output-on-failure
```

To run hardware-dependent unit tests, first connect all peripheral devices to the computer. Then,

```bash
# Re-configure to build hardware tests, then build and run
cmake -S recorder -B recorder/build -DRECORDER_BUILD_HARDWARE_TESTS=ON

# Compile using 16 CPU cores
cmake --build recorder/build -j16

# Run hardware-dependent tests only
# (change the path as needed)
export SPOTLIGHT_PROFILE_DIR=~/Spotlight/profiles/default
ctest --test-dir recorder/build --label-regex hardware --output-on-failure
```

Alternatively, to run all tests,

```bash
cmake -S recorder -B recorder/build -DRECORDER_BUILD_HARDWARE_TESTS=ON
cmake --build recorder/build -j16
export SPOTLIGHT_PROFILE_DIR=~/Spotlight/profiles/default
ctest --test-dir recorder/build --output-on-failure
```

## Trigger controller

```bash
cd spotlight-control

# Build and upload
pio test -d trigger_firmware -f test_protocol --without-testing

# Let the USB CDC re-enumerate
sleep 6

# Read results
pio test -d trigger_firmware -f test_protocol --without-building --without-uploading

cd ..
```

## Python codebase (`tools/`)

TODO