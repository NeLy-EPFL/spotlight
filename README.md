# Spotlight control

## Software dependencies and setups

### eGrabber (for Euresys frame grabber)
Download from https://www.euresys.com/en/download-area/. This should include something like:
```
egrabber-linux-offline-documentation-en-24.12.2.2194.tar.gz
egrabber-linux-sample-programs-24.12.2.16.tar.gz
egrabber-linux-x86_64-24.12.3.24.tar.gz
egrabber-release-notes-24.12.3.2195.pdf
```

Version used for development: 24.12.

Follow instruction in `egrabber-linux-x86_64-24.12.3.24/INSTALL`, namely:
```bash
cd egrabber-linux-x86_64-24.12.3.24/
sudo ./install.sh
```

**Then, don't be so fast closing the terminal!** Follow on-screen instruction to complete installation:
```
. /opt/euresys/egrabber/shell/setup_gentl_paths.sh
. /opt/euresys/egrabber/shell/select-coaxlink-producer.sh
```

Add the following to `~/.bashrc`:
```
export EURESYS_COAXLINK_GENTL64_CTI=/opt/euresys/egrabber/lib/x86_64/coaxlink.cti
```

Then, start a new shell and run:
```
/opt/euresys/egrabber/shell/select-default-producer.sh coaxlink
```

Finally, restart the computer.


### OpenCV
Follow [official instruction](https://opencv.org/get-started/). For the C++ library on Linux without building from source:
```bash
sudo apt install libopencv-dev
```

Version used: 4.6.0. To check actual version:
```bash
pkg-config --modversion opencv4
4.6.0
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

### Zaber (motion control)
#### Zaber Launcher (GUI application)
Download the `.AppImage` file from https://www.zaber.com/software; then run it directly:
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
... then, go to the FUSE github repo as directed and install FUSE following the instructions there. In brief, these are:
```bash
sudo add-apt-repository universe
sudo apt install libfuse2  # for Ubuntu 22.04
sudo apt install libfuse2t64  # for Ubuntu 24.04
```

Then run `ZaberLauncher.AppImage` again.

If the following error is encountered:
```
[13052:0208/184049.386863:FATAL:setuid_sandbox_host.cc(158)] The SUID sandbox helper binary was found, but is not configured correctly. Rather than run without sandboxing I'm aborting now. You need to make sure that /tmp/.mount_ZaberLlZtImo/chrome-sandbox is owned by root and has mode 4755.
```

... then, rerun with the `--no-sandbox` flag:
```bash
./ZaberLauncher.AppImage --no-sandbox
```

Then, follow on-screen instruction to create a connection (if not already done).

If the following error is encountered:
```
Cannot Open Connection
Cannot open serial port: Permission denied. You may need to add user to the "dialout" group to access serial ports.
You can still create the connection and resolve the issue later.
```
... then simply run:

```bash
sudo chmod ago+rw /dev/ttyACM0  # or replace with appropriate USB device
```

ALTERNATIVELY, add the current user to the `dialout` group as instructed:
```bash
sudo usermod -a -G dialout $USER
```

Then, **restart** before relaunching Zaber Launcher and retrying.

#### Zaber Motion library
Go to https://www.zaber.com/software and follow instructions under "Getting Started." For C++:
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