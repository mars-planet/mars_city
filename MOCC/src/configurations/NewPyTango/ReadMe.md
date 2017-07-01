Execute the following scripts to get a demo of the New PyTango implementation
1. `python HealthMonitorDS.py` (To start the demo Health Monitor Device Server)
2. `python demo_script.py` (to test it, in another terminal)

Working:
- A ConfigManager is created with a IP:port argument (TODO : add IP:port functionality)
- ConfigManager creates DeviceProxy object giving the device server address as the argument
- DeviceProxy dynamically binds all the functions of the Device Server to allow calls like proxy.move_forward()
- Function calls on the proxy result in REST calls