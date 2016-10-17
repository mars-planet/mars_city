#!/bin/bash

./v4l2stereo -0 /dev/video1 -1 /dev/video2 --flipleft --calibrate "6 9 24" --width 640 --height 480
