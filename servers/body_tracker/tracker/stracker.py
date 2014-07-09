from driver import lib

#Skeleton Tracker class should be instantiated only once
class SkeletonTracker(object):
    def __init__(self):
        self.addr = lib.SkeletonTracker_new()
        self.loop()

    def loop(self):
        lib.loop(self.addr)

    def shutdown(self):
        lib.shutdown(self.addr)

    def usersCount(self):
        return lib.getUsersCount(self.addr)
