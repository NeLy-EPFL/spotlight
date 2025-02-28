var g = grabbers[0];
// Start acquisition
// Stop acquisition
g.InterfacePort.set("LineSelector", "TTLIO12");
g.InterfacePort.set("LineInputToolSource", "TTLIO12");
g.RemotePort.set("TriggerSelector", "FrameStart");
g.RemotePort.set("TriggerSelector", "AcquisitionEnd");
g.RemotePort.set("TriggerSelector", "FrameStart");
g.RemotePort.set("TriggerMode", "On");
g.RemotePort.set("TriggerSource", "CXPin");
g.DevicePort.set("CameraControlMethod", "EXTERNAL");
// Start acquisition
// Stop acquisition
// Start acquisition
// Stop acquisition
g.RemotePort.set("ExposureMode", "TriggerWidth");
// Start acquisition
// Stop acquisition
g.RemotePort.set("TriggerSelector", "AcquisitionStart");
g.RemotePort.set("TriggerSelector", "AcquisitionEnd");
g.RemotePort.set("TriggerSelector", "FrameStart");
g.RemotePort.set("ExposureMode", "Off");
g.RemotePort.set("TriggerMode", "On");
g.RemotePort.set("ExposureMode", "TriggerWidth");
// Start acquisition
// Stop acquisition
// Start acquisition
// Stop acquisition
g.RemotePort.set("TriggerSelector", "AcquisitionStart");
g.RemotePort.set("TriggerSelector", "FrameStart");
g.RemotePort.set("TriggerSelector", "AcquisitionEnd");
g.RemotePort.set("TriggerSelector", "FrameStart");
// Start acquisition
// Stop acquisition
// Start acquisition
// Stop acquisition