# Using the native Python csv formatting, so as to be easily compatible
# with database management systems like MySQL, Excel
import csv
from sys import argv

class Parser(object):
    def __init__(self):
        # The directory of the database needs to be provided.
        # Provide source directory in such format:
        #   /home/Repos/eras/servers/solarstorm/database/data
        self.src = argv[1]

    def writecsv(self):
        csv_file = "traindata.csv"
        out_csv = csv.writer(open(csv_file, 'wb'))
        
        for year in range(2001, 2013):
            with open("{0}/dsd_{1}.txt".format(self.src, year), 'r') as in_txt:
                if year == 2008:
                    # The data in year 2008 seems to quite inconsistent
                    continue
                
                self.index = 0    
                for line in in_txt:
                    row_data = []
                    if line.startswith((':', '#')):
                        continue
                    values = line.split()
                    
                    if self.index == 0:
                        print "yeah"
                        self.pdate = '-'.join(values[:3])
                        self.pradioflux = values[3]
                        self.psunspotnum = values[4]
                        self.psunspotarea = values[5]
                        self.pnewregs = values[6]                        
                        
                        self.pbkgdflux_alpha = self._bkgdflux(values[8][0])
                        if (values[8] not in ("*","Unk")) :
                            self.pbkgdflux_float = float(values[8][1:])
                        else :
                            self.pbkgdflux_float = 0.0
                        
                    else :
                        print "asdf"
                        row_data.extend((self.pdate, self.pradioflux, self.psunspotnum,
                                         self.psunspotarea, self.pnewregs, self.pbkgdflux_alpha, 
                                         self.pbkgdflux_float))
                        
                        cflare = values[9]
                        mflare = values[10]
                        xflare = values[11]
                        row_data.extend((cflare, mflare, xflare))
                                                                        
                        out_csv.writerow(row_data)
                                                
                        self.pdate = '-'.join(values[:3])
                        self.pradioflux = values[3]
                        self.psunspotnum = values[4]
                        self.psunspotarea = values[5]
                        self.pnewregs = values[6]
                        self.pbkgdflux_alpha = self._bkgdflux(values[8][0])
                        if (values[8] not in ("*","Unk")) :
                            self.pbkgdflux_float = float(values[8][1:])
                        else :
                            self.pbkgdflux_float = 0.0
                        
                    self.index += 1
                    print self.index        
                    
    def _bkgdflux(self, bkgd):
        # The bkgd flux is in such string format like B1.2, A3.0, C3.6
        # So, to handle these alphanumeric values, an integer value
        # is being associated with the alphabetic part. 
        # Please note, this is an experimental method to handle these
        # values, and may be changed if a better solution is found
        flux = dict(A=1, B=2, C=3, M=4)
        if bkgd in flux:
            return flux[bkgd]
        else:
            return 0


if __name__ == '__main__':
    parse = Parser()
    parse.writecsv()
