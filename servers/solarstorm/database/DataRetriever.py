# Data Retriever
# The module can be used to retrieve data from different sources,
# to eventually prepare database for training the neural network

# Presently the DSD : Daily solar data is being downloaded,
# other data could be downloaded using the retriever method

import urllib2
import time

class DataRetriever:
    def __init__(self):
        # Download data to a user specified local directory
        # Provide destination in such formats:
        #   ubuntu : /home/simar/Repos/eras
        #   windows: C:/Repos/eras
        self.dest = raw_input("Provide download directory: ")

    def retriever(self,url,dest):
        response = urllib2.urlopen(url)
        h = response.info()
        totalsize = int(h["Content-Length"])

        print "Downloading %s bytes..." % totalsize,
        file_open = open(dest, 'wb')

        blocksize = 8192    # Reading chunks of 8192 bytes
        count = 0
        while True:
            chunk = response.read(blocksize)
            if not chunk: break
            file_open.write(chunk)
            count += 1
            if totalsize > 0:
                percent = int(count * blocksize * 100 / totalsize)
                if percent > 100: percent = 100
                print "%d%%" % percent,
                if percent < 100:
                    print "\b",
                else:
                    print "Done."
        file_open.flush()
        file_open.close()

    def dsd_retriever(self):
        for n in range(1996,2013):
            url = "http://www.swpc.noaa.gov/ftpdir/warehouse/"+str(n)+\
                "/"+str(n)+"_DSD.txt"

            # Constructing output as example : destination/dsd_2011.txt
            dest = str(self.dest) + "/dsd_" + str(n) +".txt"
            print "\nDownloading DSD %s data :" % n
            self.retriever(url,dest)
            #Providing sleep time to prevent forced session termination
            time.sleep(3)


if __name__ == '__main__':
    retrieve = DataRetriever()
    retrieve.dsd_retriever()
