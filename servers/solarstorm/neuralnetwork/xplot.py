from pybrain.tools.customxml.networkreader import NetworkReader
import pylab as pl
import numpy as np


class XPlot(object):

    def xactual(self):
        with open('xtraindata.csv') as tf:
            xactual = []
            for line in tf:
                data = [x for x in line.strip().split(',') if x]
                xactual.append(float(data[12]))
        return xactual

    def xforecast(self):
        net = NetworkReader.readFrom('xtrainedinfo.xml')
        activate_in = []
        with open('xtraindata.csv') as tf:
            xforecast = []
            for line in tf:
                data = [x for x in line.strip().split(',') if x]
                for i in range(1, 10):
                    activate_in.append(float(data[i]))
                # print activate_in
                if float(net.activate((activate_in))) > 4.84e-06:
                    xforecast.append(2)
                elif float(net.activate((activate_in))) > 3.5e-06:
                    xforecast.append(1)
                else:
                    xforecast.append(0)
                activate_in = []
        return xforecast

    def plot(self):
        xactual = self.xactual()
        xforecast = self.xforecast()
        days = np.linspace(1, 3264, 3264)

        print "Plotting..."
        pl.plot(days, xactual, 'ob')
        pl.plot(days, xforecast, '-r')
        pl.plot(days, xforecast, '.r')
        pl.savefig('historicalplot.png')
        pl.show()

if __name__ == '__main__':
    xp = XPlot()
    xp.plot()
