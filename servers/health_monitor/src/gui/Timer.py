from threading import Thread, Event


class Timer(Thread):

    def __init__(self, *args, **kwargs):
        super(Timer, self).__init__(*args, **kwargs)
        self._stop = Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()
