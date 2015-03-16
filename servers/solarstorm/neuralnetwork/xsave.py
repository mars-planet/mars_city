import csv
import urllib
import datetime as dt
from pybrain.tools.customxml.networkreader import NetworkReader

data = []
page = urllib.urlopen('http://www.swpc.noaa.gov/ftpdir/indices/DSD.txt')
for line in page:
    if line.startswith((':', '#')):
        continue
    data.append(line.split())


def bkgdflux(bkgd):
    flux = dict(A=1, B=2, C=3, M=4)
    if bkgd in flux:
        return flux[bkgd]
    else:
        return 0

values = data[-1]

# Parse all the current values
radioflux = values[3]
sunspotnum = values[4]
sunspotarea = values[5]
newregs = values[6]
bkgdflux_alpha = bkgdflux(values[8][0])

if values[8] not in ("*", "Unk"):
    bkgdflux_float = float(values[8][1:])
else:
    bkgdflux_float = 0.0

cflare = values[9]
mflare = values[10]
xflare = values[11]

input_data = []

input_data.extend((radioflux, sunspotnum, sunspotarea, newregs, bkgdflux_alpha,
                   bkgdflux_float, cflare, mflare, xflare))


net = NetworkReader.readFrom('xtrainedinfo.xml')
xvalue = net.activate((input_data))

if xvalue[0] > 4.84e-06:
    xforecast = 2
elif xvalue[0] > 3.5e-06:
    xforecast = 1
else:
    xforecast = 0

today = dt.date.today()
tomorrow = today + dt.timedelta(days=1)
rowdata = []
print "Writing tommorow's forecast..."

forecast = csv.writer(open("xforecast.csv", 'a'))
rowdata.extend((tomorrow, xforecast))
forecast.writerow(rowdata)
