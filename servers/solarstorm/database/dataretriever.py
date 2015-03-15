# Data Retriever
# The module can be used to retrieve data from different sources,
# to eventually prepare database for training the neural network

# Presently the DSD : Daily solar data is being downloaded,
# other data could be downloaded using the retriever method

import urllib2
from time import sleep
from sys import argv


class DataRetriever:

    def __init__(self):
        # Download data to a user specified local directory
        # Provide destination in such formats:
        #   ubuntu : /home/simar/Repos/eras
        #   windows: C:/Repos/eras
        self.dest = argv[1]

    def retriever(self, url, dest):
        response = urllib2.urlopen(url)
        h = response.info()
        totalsize = int(h["Content-Length"])

        print "Downloading {0} bytes...".format(totalsize),
        file_open = open(dest, 'wb')

        blocksize = 8192    # Reading chunks of 8192 bytes
        count = 0
        while True:
            chunk = response.read(blocksize)
            if not chunk:
                break
            file_open.write(chunk)
            count += 1
            if totalsize > 0:
                percent = int(count * blocksize * 100 / totalsize)
                percent = min(percent, 100)
                print "{0}%".format(percent),
                if percent < 100:
                    print "\b",
                else:
                    print "Done."
        file_open.flush()
        file_open.close()

    def dsd_retriever(self):
        for n in range(1996, 2013):
            url = "http://www.swpc.noaa.gov/ftpdir/warehouse/{0}/{0}_DSD.txt"
            url = url.format(n)

            # Constructing output as example : destination/dsd_2011.txt
            dest = "{0}/dsd_{1}.txt".format(self.dest, n)
            print "\nDownloading DSD {0} data :".format(n)
            self.retriever(url, dest)
            # Providing sleep time to prevent forced session termination
            sleep(3)


if __name__ == '__main__':
    retrieve = DataRetriever()
    retrieve.dsd_retriever()
