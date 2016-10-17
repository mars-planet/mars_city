#!/bin/bash

# Generates packaging

rm -f Makefile rpmpackage/*.spec

packagemonkey -n "v4l2stereo" --cmd --dir "." -l "gpl3" -e "Bob Mottram (4096 bits) <bob@robotics.uk.to>" --brief "Utility for stereoscopic vision" --desc "Enables the generation of depth maps from stereo vision" --homepage "https://github.com/fuzzgun/v4l2stereo" --repository "https://github.com/fuzzgun/v4l2stereo.git" --section "utils" --version "0.1" --categories "Utility/ConsoleOnly" --compile "src/calibration/*.cpp src/elas/*.cpp -msse3 -fopenmp -I/usr/include/opencv `pkg-config opencv --cflags --libs`" --builddeb "libopencv-dev" --dependsdeb "libopencv" --buildarch "libopencv" --dependsarch "libopencv" --dependsrpm "libopencv-devel" --dependsrpmdistro "libopencv" --buildrpm "libopencv-devel" --buildrpmdistro "libopencv-devel"
