# Software dependencies

Install steps for the libraries and vendor SDKs the recorder builds against, plus
the GUI tools used to configure the hardware. Most steps are ported from the
original setup notes and kept deliberately verbatim; version numbers are the ones
used during development.

> [!NOTE]
> Some build dependencies are now fetched automatically by CMake (ArduinoJson, and
> GoogleTest for the tests) and the trigger firmware is built with PlatformIO
> rather than the Arduino IDE. Those changes are flagged inline below. See
> [Building and installing](building.md) for the build commands.

## eGrabber (for the Euresys frame grabber)

Download both eGrabber and Memento (a debugging tool) from
https://www.euresys.com/en/download-area/. This should include something like:

```
# eGrabber
egrabber-linux-offline-documentation-en-24.12.2.2194.tar.gz
egrabber-linux-sample-programs-24.12.2.16.tar.gz
egrabber-linux-x86_64-24.12.3.24.tar.gz
egrabber-release-notes-24.12.3.2195.pdf

# Memento
memento-linux-offline-documentation-en-24.12.0.6056.tar.gz
memento-linux-x86_64-24.12.0.3.tar.gz
memento-release-notes-24.12.0.6056.pdf
```

Version used for development: 24.12 (for both).

> [!IMPORTANT]
> You must install Memento first, before eGrabber!

Use the provided installer:

```bash
cd memento-linux-x86_64-24.12.0.3/
sudo ./install.sh
```

It's possible that you are missing some dependencies. Read the output carefully and
follow any instructions.

Then, to install eGrabber, follow the instructions in
`egrabber-linux-x86_64-24.12.3.24/INSTALL`, namely:

```bash
cd egrabber-linux-x86_64-24.12.3.24/
sudo ./install.sh
```

> [!IMPORTANT]
> Upon installation, don't be too fast closing the terminal! Read the output
> carefully and follow on-screen instructions to complete the process. In brief:
> ```
> . /opt/euresys/egrabber/shell/setup_gentl_paths.sh
> . /opt/euresys/egrabber/shell/select-coaxlink-producer.sh
> ```
>
> Add the following to `~/.bashrc`:
> ```
> export EURESYS_COAXLINK_GENTL64_CTI=/opt/euresys/egrabber/lib/x86_64/coaxlink.cti
> ```
>
> Then, start a new shell and run:
> ```
> /opt/euresys/egrabber/shell/select-default-producer.sh coaxlink
> ```
>
> Finally, restart the computer.

Then move on to [eGrabber and JAI camera configuration](egrabber_jai_config.md) to
finish configuring the frame grabber and the camera.

### Troubleshooting

In case of problems, the following log files are worth looking into:

- `/opt/euresys/egrabber/install.log`.
- `sudo /opt/euresys/egrabber/shell/check-install.sh` output.
- `sudo dmesg` output.
- `sudo lspci -vvv` output.
- A highest-verbosity Memento dump captured while you start eGrabber Studio (start
  Memento, then launch eGrabber).

> [!TIP]
> Given the issue might be hardware-related, it might help to really power-cycle the
> computer instead of restarting (i.e. not just `sudo reboot now`). Specifically:
> turn the computer off, cut the power supply, press the On button to discharge the
> capacitors, wait a bit to be extra sure, then turn the power supply back on and
> start the computer.

> [!NOTE]
> I encountered a problem where, upon booting the OS, the four LEDs on the frame
> grabber bracket stayed solid orange. Based on the frame grabber manual, this
> indicates that the frame grabber never exited "system booting" mode. With
> assistance from Euresys support, we narrowed the problem down to the following
> error messages from Memento:
> ```
> [    2.902599] memento: no symbol version for module_layout
> [    2.902645] memento: disagrees about version of symbol devmap_managed_key
> [    2.902646] memento: Unknown symbol devmap_managed_key (err -22)
> ```
> ... and the solution was to uninstall eGrabber, uninstall Memento, reboot the
> machine (following the procedure above), install Memento (latest version, i.e.
> 24.12), and install eGrabber (latest version, i.e. 24.12) **in this order**.

If nothing works, contact Euresys support. They (specifically Mr. Julien Weber)
were very helpful.

## OpenCV

Follow the [official instructions](https://opencv.org/get-started/). For the C++
library on Linux without building from source:

```bash
sudo apt install libopencv-dev
```

Version used: 4.6.0. To check the actual version:

```bash
pkg-config --modversion opencv4
# 4.6.0
```

Boilerplate code and `CMakeLists.txt` to get started:

```bash
cd opt/examples/opencv_cpp_boilerplate
mkdir build
cd build
cmake ..
make
./main <path-to-some-image>  # the image should display
```

## Zaber (motion control)

### Zaber Launcher (GUI application)

Download the `.AppImage` file from https://www.zaber.com/software, then run it
directly:

```bash
chmod ago+x ZaberLauncher.AppImage
./ZaberLauncher.AppImage
```

If the following error is encountered:

```
AppImages require FUSE to run.
You might still be able to extract the contents of this AppImage
if you run it with the --appimage-extract option.
See https://github.com/AppImage/AppImageKit/wiki/FUSE
for more information
```

... then go to the FUSE GitHub repo as directed and install FUSE following the
instructions there. In brief:

```bash
sudo add-apt-repository universe
sudo apt install libfuse2     # for Ubuntu 22.04
sudo apt install libfuse2t64  # for Ubuntu 24.04
```

Then run `ZaberLauncher.AppImage` again.

If the following error is encountered:

```
[...]:FATAL:setuid_sandbox_host.cc(158)] The SUID sandbox helper binary was found, but is not configured correctly. Rather than run without sandboxing I'm aborting now. You need to make sure that /tmp/.mount_ZaberLlZtImo/chrome-sandbox is owned by root and has mode 4755.
```

... then rerun with the `--no-sandbox` flag:

```bash
./ZaberLauncher.AppImage --no-sandbox
```

Then follow on-screen instructions to create a connection (if not already done).

If the following error is encountered:

```
Cannot Open Connection
Cannot open serial port: Permission denied. You may need to add user to the "dialout" group to access serial ports.
You can still create the connection and resolve the issue later.
```

... then simply run:

```bash
sudo chmod ago+rw /dev/ttyACM0  # or replace with the appropriate USB device
```

ALTERNATIVELY, add the current user to the `dialout` group as instructed:

```bash
sudo usermod -a -G dialout $USER
```

Then **restart** before relaunching Zaber Launcher and retrying.

### Zaber Motion library

Go to https://www.zaber.com/software and follow the instructions under "Getting
Started." For C++:

```bash
sudo dpkg -i ZaberMotionCppInstaller-7.4.0-Linux_x64.deb
```

Boilerplate code and `CMakeLists.txt` to get started:

```bash
cd examples/zaber_cpp_boilerplate/
mkdir build
cd build
cmake ..
make
./zaber
```

## Qt (GUI framework)

> [!IMPORTANT]
> The Qt framework is available under both open-source and commercial licenses. To
> use the open-source license, our project must be licensed under GPL or LGPL. We
> use GPL v3.

> [!NOTE]
> You will need a Qt account, which you can sign up for free.

Download the Qt online installer (with the open-source license) from
https://www.qt.io/download-open-source. Give every user execute permission to the
downloaded `.run` file, and run it:

```bash
chmod ago+x qt-online-installer-linux-x64-4.8.1.run
./qt-online-installer-linux-x64-4.8.1.run
```

Then follow the on-screen instructions. You might need to install some
dependencies. For example:

```bash
sudo apt install libxcb-cursor0 libxcb-cursor-dev
```

You might need to save the Qt installation path as an environment variable so that
CMake can find it. Add the following to `~/.bashrc` (version used: 6.8.2):

```bash
# Qt6
export Qt6_DIR="${HOME}/Qt/6.8.2/gcc_64/lib/cmake/Qt6"
```

Alternatively, adding `set(Qt6_DIR "${HOME}/Qt/6.8.2/gcc_64/lib/cmake/Qt6")` before
`find_package` in the `CMakeLists.txt` file probably also works.

I also installed Qt Creator, a UI development tool.

## PCO camera software

Go to the "Software" tab on the
[product page for the pco.panda 4.2 sCMOS camera](https://www.excelitas.com/product/pcopanda-42-scmos-camera)
on the Excelitas website. This gives you two sub-tabs: "pco.panda 4.2 USB Firmware"
and "PCO Camera Software". The former is used in case you want to update the camera
firmware; it only works on Windows. To just use the camera, go to the latter.

Once on the ["PCO Camera Software"](https://www.excelitas.com/product-category/pco-camera-software)
page, go to ["PCO Software Development Kits"](https://www.excelitas.com/product/pco-software-development-kits).
("PCO Camera Control Software" might also be useful; it looks like a GUI program but
it only runs on Windows so I didn't try.) Under the "General SDK" tab, download
pco.sdk, pco.recorder, and pco.runtime. Under the "C++" tab, download pco.cpp. Take
care to download the file for the right computer architecture — in this case AMD
(x86_64).

Then unzip each of the 4 downloaded files. This should give you 4 `.deb` files.
Install each of them with `dpkg`:

```bash
sudo dpkg -i pco.sdk_1.33.0_amd64.deb
sudo dpkg -i pco.runtime_3.5.0_amd64.deb
sudo dpkg -i pco.recorder_3.5.0_amd64.deb
sudo dpkg -i pco.cpp_1.4.0_amd64.deb
```

> [!NOTE]
> I don't know if all four are actually required or if the order of installation
> matters, but this is what made sense to me and what I did.

This should install a bunch of files under
`/opt/pco/{pco.cpp,pco.recorder,pco.runtime,pco.sdk}`. Among these are some useful
sample programs under `/opt/pco/pco.cpp/samples`. In particular, `ImageViewer` is a
nice GUI program.

> [!IMPORTANT]
> The muscle camera is now driven **in-process** by the recorder, which compiles
> the bundled PCO SDK sources directly. `/opt/pco/pco.cpp/lib/` ships its own
> `libQt6*` and `libicu*`; these must never end up ahead of the system Qt on the
> recorder's library search path. The recorder's CMake handles this (it keeps the
> system Qt lib dir ahead of the PCO lib dir on the rpath). See
> [Building and installing](building.md) and the
> [architecture overview](../architecture.md).

## spdlog (C++ logging library)

Install using the Debian package manager:

```bash
sudo apt install libspdlog-dev
```

See the hello-world example at `opt/examples/spdlog_boilerplate/`:

```bash
cd opt/examples/spdlog_boilerplate/
mkdir build
cd build
cmake ..
make
./example
```

> [!NOTE]
> The version from the Debian package manager is an older version that does not
> implement Mapped Diagnostic Context (MDC). Since I don't really need it, I used
> the `apt`-managed version for simplicity and removed the MDC-related code from
> `example.cpp` provided in spdlog's GitHub repo.

## yaml-cpp

Download `yaml-cpp` from https://github.com/jbeder/yaml-cpp (v0.8.0 for me). Then do
the usual:

```bash
unzip yaml-cpp-0.8.0
cd yaml-cpp-0.8.0
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

## GoogleTest (C++ unit testing)

> [!NOTE]
> The `recorder` and `comm_protocol` builds now fetch GoogleTest automatically via
> CMake `FetchContent` when configured standalone, so a manual system-wide install
> is generally **not** required. The steps below are kept for reference / offline
> use.

Download from https://github.com/google/googletest and extract the downloaded file:

```bash
tar xf googletest-1.16.0.tar.gz  # change version accordingly
cd googletest-1.16.0/
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make
sudo make install
```

## PlatformIO (trigger firmware)

The trigger firmware (`trigger_firmware/`) is built and uploaded with **PlatformIO**
(we use PlatformIO, not the Arduino IDE). Install it either as the **PlatformIO IDE**
(the official VS Code extension) or as the standalone CLI (`pip install platformio`).
Both provide the `pio` command used in
[Building and installing](building.md#trigger_firmware). PlatformIO resolves the
board, toolchain, and library dependencies from `trigger_firmware/platformio.ini`
automatically — no separate IDE or manual board-package install is needed.

### USB-upload troubleshooting

If uploading the firmware to the board fails with a USB-permission error:

```
dfu-util: Cannot open DFU device 2341:0070 found on devnum 21 (LIBUSB_ERROR_ACCESS)
dfu-util: No DFU capable USB device available
Failed uploading: uploading error: exit status 74
```

... then check if `dfu-util` is installed. If not:

```bash
sudo apt install dfu-util
# If prompted by the output, you might need to run the following first:
# sudo apt --fix-broken install
```

... then create a file `/etc/udev/rules.d/50-arduino.rules` with the following line:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="2341", MODE="0666"
```

(This allows all users to access Arduino devices with Vendor ID 2341.)

Then reload these rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

You should see the board detected when you run:

```bash
dfu-util -l
```

Then unplug and replug the board and try again. (See also the Arduino entry in
[Troubleshooting](../troubleshooting.md).)

## Python

The offline tools use **uv** for package management; see
[Building and installing](building.md#python-tools-spotlight-tools) and the
[`spotlight-tools`](https://github.com/NeLy-EPFL/spotlight-tools) repository.

> [!NOTE]
> The original setup used a plain `venv` for an in-repo Python environment:
> ```bash
> sudo apt install python3.12-venv  # change version if needed
> python -m venv python_env
> source python_env/bin/activate
> ```
> Avoid Conda: in my experience it made `spdlog` and `googletest` incompatible
> somehow.
