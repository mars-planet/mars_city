# Using the native Python csv formatting, so as to be easily compatible
# with database management systems like MySQL, Excel
import csv
from sys import argv

class Parser:
    def __init__(self):
        # The directory of the database needs to be provided.
        # Provide source directory in such format:
        #   /home/Repos/eras/servers/solarstorm/database/data
        self.src = argv[1]

    def writecsv(self):
        csv_file = "traindata.csv"
        out_csv = csv.writer(open(csv_file, 'wb'))
        
        for n in range(2001, 2013):
            with open("{0}/dsd_{1}.txt".format(self.src, n), 'r') as in_txt:
                if n == 2008:
                    continue
                for line in in_txt:
                    row_data = []
                    if line.startswith((':', '#')):
                        continue
                    values = line.split()
                    
                    date = '/'.join(values[:3])
                    radioflux = values[3]
                    sunspotnum = values[4]
                    sunspotarea = values[5]
                    newregs = values[6]
                    row_data.extend((date,radioflux,sunspotnum,sunspotarea,newregs))
                    
                    # Currently assigning 0 to the missing values (*),
                    # but can be changed later to a better substitute value
                    bkgdflux_alpha = self.bkgdflux(values[8][0])
                    bkgdflux_float = float(values[8][1:]) if not "*" else 0
                    row_data.extend((bkgdflux_alpha, bkgdflux_float))
                    
                    # Using the C,M,X classification and not using the
                    # optical S,1,2,3 classification as it is no longer
                    # widely used
                    cflare = values[9]
                    mflare = values[10]
                    xflare = values[11]
                    row_data.extend((cflare,mflare,xflare))
                    
                    out_csv.writerow(row_data)
                    
    def bkgdflux(self, bkgd):
        # The bkgd flux is in such string format like B1.2, A3.0, C3.6
        # So, to handle these alphanumeric values, an integer value
        # is being associated with the alphabetic part. 
        # Please note, this is an experimental method to handle these
        # values, and may be changed if a better solution is found
        flux = {"A" : 1, "B" : 2, "C" : 3, "M" : 4}
        if bkgd in flux:
            return flux[bkgd]
        else:
            return 0


if __name__ == '__main__':
    parse = Parser()
    parse.writecsv()
