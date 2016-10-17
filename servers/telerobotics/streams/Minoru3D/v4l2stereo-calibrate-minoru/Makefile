APP=v4l2stereo
VERSION=0.1
RELEASE=1
ARCH_TYPE=`uname -m`
PREFIX?=/usr/local

all:
	g++ -Wall -pedantic -O3 -std=c++11 -o ${APP} src/*.cpp -Isrc src/calibration/*.cpp src/elas/*.cpp -msse3 -fopenmp -I/usr/include/opencv -I/usr/include/opencv  -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab  
debug:
	g++ -Wall -pedantic -g -std=c++11 -o ${APP} src/*.cpp -Isrc src/calibration/*.cpp src/elas/*.cpp -msse3 -fopenmp -I/usr/include/opencv -I/usr/include/opencv  -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab  
source:
	tar -cvf ../${APP}_${VERSION}.orig.tar ../${APP}-${VERSION} --exclude-vcs
	gzip -f9n ../${APP}_${VERSION}.orig.tar
install:
	mkdir -p ${DESTDIR}/usr
	mkdir -p ${DESTDIR}${PREFIX}
	mkdir -p ${DESTDIR}${PREFIX}/bin
	install -m 755 --strip ${APP} ${DESTDIR}${PREFIX}/bin
	mkdir -m 755 -p ${DESTDIR}${PREFIX}/share
	mkdir -m 755 -p ${DESTDIR}${PREFIX}/share/man
	mkdir -m 755 -p ${DESTDIR}${PREFIX}/share/man/man1
	install -m 644 man/${APP}.1.gz ${DESTDIR}${PREFIX}/share/man/man1
uninstall:
	rm -f ${PREFIX}/share/man/man1/${APP}.1.gz
	rm -rf ${PREFIX}/share/${APP}
	rm -f ${PREFIX}/bin/${APP}
clean:
	rm -f ${APP} \#* \.#* gnuplot* *.png debian/*.substvars debian/*.log
	rm -fr deb.* debian/${APP} rpmpackage/${ARCH_TYPE}
	rm -f ../${APP}*.deb ../${APP}*.changes ../${APP}*.asc ../${APP}*.dsc
	rm -f rpmpackage/*.src.rpm archpackage/*.gz archpackage/*.xz
	rm -f puppypackage/*.gz puppypackage/*.pet slackpackage/*.txz
