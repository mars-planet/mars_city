SETUP
=============
1. Build LibOVR as shared object.

	To do this, copy the provided Makefile into the OculusSDK/LibOVR folder and type make

2. Setup pyOVR

	Copy the generated libovr.so from OculusSDK/LibOVR/Lib/.../libovr.so into the folder where pyOVR.py resides, or
	modify the LIBOVR_PATH constant in the pyOVR.py script for any custom path you wish to use (e.g. /usr/lib64).
	If you modify it, remeber that the path can't be relative! It has to be the full path to the library.

	To test the bindings, write:
	>> python3 TestSimple.py
	>> python3 TestRiftDevice.py

	The first tests the C-interface wrapper, the second the custom Python-wrapper class.
	If successfully, you should see the output of the Quaternion values, Rift's FusionSensor provides.
