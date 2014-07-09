SkeletonTracker.so:SkeletonTracker.o
	g++ -shared -Wl,-soname,SkeletonTracker.so -o SkeletonTracker.so  SkeletonTracker.o -lglut -lOpenNI2 -lNiTE2 -L./OpenNI2/Redist -L./NiTE2/Redist -L./

SkeletonTracker.o: SkeletonTracker.cpp
	g++ -c -fPIC -I./NiTE2/Include -I./OpenNI2/Include SkeletonTracker.cpp -o SkeletonTracker.o  -lglut -lOpenNI2 -lNiTE2 -L./OpenNI2/Redist -L./NiTE2/Redist -L./

