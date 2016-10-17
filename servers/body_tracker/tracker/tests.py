import unittest

import time
from PyTango import DeviceProxy
import tracker
import threading


class TestBodyTracker(unittest.TestCase):
    def test_user_height(self):
        test_file = 'test_files\\height_test.json'
        tracker_thread = threading.Thread(target=tracker.Tracker,
                                          args=('eras-1',
                                                None,
                                                None,
                                                test_file
                                                )
                                          )
        tracker_thread.daemon = True
        tracker_thread.start()

        time.sleep(2)

        # wait for end of simulation
        while not tracker.sim_cond.is_set():
            tracker.sim_cond.wait()

        time.sleep(2)

        tango_test = DeviceProxy("c3/MAC/eras-1")
        height = tango_test.get_height()
        self.assertEqual("%.5f" % height, "%.5f" % 1.72074513019)

        # TBD: stop the tracker_thread
        # tracker_thread._stop()

    # def test_step_estimation(self):
    #    tango_test = DeviceProxy("c3/MAC/eras-1")
    #    # TBD

    # def test_skeletal_tracker(self):
    #    tango_test = DeviceProxy("c3/MAC/eras-1")
    #    # TBD

    # def test_hand_gestures(self):
    #    tango_test = DeviceProxy("c3/MAC/eras-1")
    #    # TBD

if __name__ == '__main__':
    unittest.main()
