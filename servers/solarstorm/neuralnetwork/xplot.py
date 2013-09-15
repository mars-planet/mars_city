from pybrain.tools.customxml.networkreader import NetworkReader
import pylab as pl
import numpy as np

class XPlot(object):
    def xactual(self):
        tf = open('xtraindata.csv','r')
        xactual = []
        for line in tf.readlines():
            data = [x for x in line.strip().split(',') if x != '']
            xactual.append(float(data[12]))

        tf.close()
        return xactual

    def xforecast(self):
        tf = open('xtraindata.csv','r')
        net = NetworkReader.readFrom('xtrainedinfo.xml')

        xforecast = []
        activate_in = []

        for line in tf.readlines():
            data = [x for x in line.strip().split(',') if x != '']
            for i in range(1,10):
                activate_in.append(float(data[i]))
            # print activate_in
            if float(net.activate((activate_in))) > 4.84e-06:
                xforecast.append(2)
            elif float(net.activate((activate_in))) > 3.5e-06:
                xforecast.append(1)
            else:
                xforecast.append(0)
            activate_in = []

        tf.close()
        return xforecast

    def plot(self):
        xactual = self.xactual()
        xforecast = self.xforecast()
        days = np.linspace(1, 3264, 3264)
        
        print "Plotting..."
        pl.plot(days,xactual, 'ob')
        pl.plot(days,xforecast, '-r')
        pl.plot(days,xforecast, '.r')
        pl.show()

if __name__ == '__main__':
    xp = XPlot()
    xp.plot()
