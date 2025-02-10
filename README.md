# Spotlight control

## Software dependencies and setups

### eGrabber (for Euresys frame grabber)
Download from https://www.euresys.com/en/download-area/

Tested version: 24.12


### OpenCV
Follow [official instruction](https://opencv.org/get-started/). For the C++ library on Linux without building from source:
```bash
apt install libopencv-dev
```

Version: 4.6.0. To check actual version:
```bash
$ pkg-config --modversion opencv4
4.6.0
```

To verify install:
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