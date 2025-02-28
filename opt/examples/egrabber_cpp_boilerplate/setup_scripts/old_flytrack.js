var g = grabbers[0];

g.InterfacePort.execute("CxpPoCxpAuto");

g.RemotePort.set("TriggerSelector", "AcquisitionStart");

g.RemotePort.set("TriggerMode", "On");

g.RemotePort.set("TriggerSource", "CXPin");

g.RemotePort.set("TriggerSelector", "FrameStart");

if(g.RemotePort.get("ExposureMode")==="TriggerWidth"){

                g.RemotePort.set("ExposureMode", "Off");

                }

g.RemotePort.set("TriggerMode", "On");

g.RemotePort.set("TriggerSource", "CXPin");

g.RemotePort.set("ExposureMode", "TriggerWidth");

 

g.RemotePort.set("LinkConfig", "CXP6_X4");

 

g.DevicePort.set("CameraControlMethod", "RG");

g.DevicePort.set("CycleTriggerSource", "Immediate");

 

g.InterfacePort.set("LineSelector", "TTLIO11");

g.InterfacePort.set("LineSource", "Device0Strobe");

g.InterfacePort.set("LineMode", "Output");

g.InterfacePort.set("LineSelector", "TTLIO12");

g.InterfacePort.set("LineSource", "Device0Strobe");

g.InterfacePort.set("LineMode", "Output");

g.InterfacePort.set("LineSourceDivisionFactor", "4");

 

g.RemotePort.set("Width", 1984);

g.RemotePort.set("Height", 1024);

g.RemotePort.set("OffsetX", 256);

g.RemotePort.set("OffsetY", 690);

g.RemotePort.set("Gain", "2");
