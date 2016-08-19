Instructions:

- Make sure your PyTango is up and running.

- Run file "reg_server.py" by 'python reg_server.py'

- Now run the time server by 'python time.py test'

- Open python console (terminal) and access the server by these commands

&gt;&gt;&gt; import PyTango

&gt;&gt;&gt; my_clock = PyTango.DeviceProxy("test/timing/1")

&gt;&gt;&gt; my_clock.time

&gt;&gt;&gt; my_clock.strftime("%H:%M:%S")