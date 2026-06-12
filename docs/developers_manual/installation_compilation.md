# Installation & compilation

## Step 1: Clone Git repositories

```bash
# Clone C++ and embedded codebase
git clone git@github.com:NeLy-EPFL/spotlight-control.git

# Clone Python tools
git clone git@github.com:NeLy-EPFL/spotlight-tools.git
```

## Step 2: Build recorder programs

```bash
cd spotlight-control

# Generate Makefile using CMake
cmake -S recorder -B recorder/build

# Run Makefile using 16 CPU cores
cmake --build recorder/build -j 16

# Install executables to recorder/bin
cmake --install recorder/build

cd ..
```

This installs the executables to `recorder/bin/`. When running the programs, either use paths to these executables explicitly (e.g., `/path/to/spotlight-control/recorder/bin/run-spotlight -p ... -a ...`), or add `recorder/build` to the front of the `$PATH` environment variables (add the line `export PATH="/path/to/spotlight-control/recorder/bin:$PATH"`. Note that this changes the default version of the programs when you launch them from the terminal).

## Step 3: Build and upload trigger firmware

Plug the Arduino Nano ESP32 board into the computer, then:

```bash
cd spotlight-control

# Build
pio run -d trigger_firmware

# Upload to device
pio run -d trigger_firmware -t upload

cd ..
```

## Step 4: Install `spotlight-tools`

```bash
cd spotlight-tools
uv sync  # see https://docs.astral.sh/uv/ if uv is not installed
cd ..
```