# This script check if the two .dll files needed for gesture recognition
# are updated and in the correct location.
# If they are not, it try to find them, to copy both files in the right folder

from __future__ import print_function

import sys
import glob
import platform
import shutil
import os
from os import path

KINECT_INTERACTION_FILENAME = 'KinectInteraction180_32.dll'
KINECT_GESTURE_RECOGNIZER_FILENAME = 'KinectGestureRecognizer.dll'
KINECT_GESTURE_RECOGNIZER_DEF_DIR = '.\\KinectGestureRecognizer\\Release'


def check_dlls():
    def is_os_64bit():
        return platform.machine().endswith('64')

    def get_kinect_interaction_def_dir():
        prog_files = None
        if is_os_64bit():
            prog_files = os.environ['ProgramW6432']
        else:
            prog_files = os.environ['ProgramFiles(x86)']

        try:
            tmp_path = glob.glob(path.join(prog_files,
                                           'Microsoft SDKs\\Kinect\\Developer Toolkit v1*\\'))[0]
        except:
            return None
        return path.join(tmp_path, 'bin')

    flag = False

    interaction_path1 = path.join(KINECT_GESTURE_RECOGNIZER_DEF_DIR,
                                  KINECT_INTERACTION_FILENAME)
    try:
        interaction_path2 = path.join(get_kinect_interaction_def_dir(),
                                      KINECT_INTERACTION_FILENAME)
    except:
        print('Error: Kinect Developer Toolkit not found')
        return False

    if path.isfile(KINECT_INTERACTION_FILENAME):
        # KinectInteraction180_32.dll exists and it is in the right place
        flag = True
    elif path.isfile(interaction_path1):
        # KinectInteraction180_32.dll exists; it must be copied in the right folder
        shutil.copy(interaction_path1, '.')
        flag = True
    elif path.isfile(interaction_path2):
        # Same as the previous case
        shutil.copy(interaction_path2, '.')
        flag = True
    else:
        # File does not exists
        flag = False

    if not flag:
        print('Error: ' + KINECT_INTERACTION_FILENAME + ' not found.')
        return False

    flag = False

    gesture_path = path.abspath(path.join(KINECT_GESTURE_RECOGNIZER_DEF_DIR, KINECT_GESTURE_RECOGNIZER_FILENAME))

    if path.isfile(KINECT_GESTURE_RECOGNIZER_FILENAME):
        # KinectGestureRecognizer.dll exists and it is in the right place
        flag = True

        # Check for an updated version
        last_mod_in_dir = path.getmtime(KINECT_GESTURE_RECOGNIZER_FILENAME)
        last_mod_gest_path = path.getmtime(gesture_path)
        if last_mod_gest_path > last_mod_in_dir:
            # An updated version of KinectGestureRecognizer.dll is available
            try:
                shutil.copy(gesture_path, '.')
            except IOError, e:
                print("WARNING: error in updating " + KINECT_GESTURE_RECOGNIZER_FILENAME, file=sys.stderr)
    elif path.isfile(gesture_path):
        # KinectGestureRecognizer.dll exists, but it must be copied in the right folder
        shutil.copy(gesture_path, '.')
        flag = True
    else:
        # File does not exists
        flag = False

    if not flag:
        print('Error: ' + KINECT_GESTURE_RECOGNIZER_FILENAME + ' not found.')
        print('Have you compiled and built the C++ gesture recognition library?')
        return False

    return True

if not check_dlls():
    sys.exit(1)
