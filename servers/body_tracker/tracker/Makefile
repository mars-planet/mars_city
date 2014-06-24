SkeletonTracker.so:SkeletonTracker.o
	g++ -shared -Wl,-soname,SkeletonTracker.so -o SkeletonTracker.so  SkeletonTracker.o -lglut -lOpenNI2 -lNiTE2 -L../../../OpenNI-2.1.0-x64/Redist -L../../../NiTE-2.0.0/Redist -L./

SkeletonTracker.o: SkeletonTracker.cpp
	g++ -c -fPIC -I../../../NiTE-2.0.0/Include -I../../../OpenNI-2.1.0-x64/Include SkeletonTracker.cpp -o SkeletonTracker.o  -lglut -lOpenNI2 -lNiTE2 -L../../../OpenNI-2.1.0-x64/Redist -L../../../NiTE-2.0.0/Redist -L./

