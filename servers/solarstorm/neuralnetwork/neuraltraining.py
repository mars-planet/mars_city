from pybrain.datasets import SupervisedDataSet
from pybrain.structure.networks import Network
from pybrain.structure.networks import FeedForwardNetwork
from pybrain.structure.modules import *
from pybrain.structure.connections import *
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.tools.customxml.networkwriter import NetworkWriter
import csv
#import pickle


class NeuralNetworkTraining(object):

    def getdata(self):
        # Currently, the Supervised dataset is being generated. The use
        # of other dataset types like Classification datasets will also
        # be investigated where it is relevant.
        ds = SupervisedDataSet(9, 3)
        tf = open('traindata.csv')
        for line in tf.readlines():
            data = [x for x in line.strip().split(',') if x != '']
            indata = tuple(data[1:10])
            outdata = tuple(data[10:])
            ds.addSample(indata, outdata)
        tf.close()
        return ds

    def neuralnetworktrain(self):
        dataset = self.getdata()

        # Constructing a multiple output neural network.
        # Other neural network architectures will also be experimented,
        # like using different single output neural networks.
        net = FeedForwardNetwork()
        inp = LinearLayer(9)
        h1 = SigmoidLayer(20)
        h2 = TanhLayer(10)
        outp = LinearLayer(3)

        # Adding the modules to the architecture
        net.addOutputModule(outp)
        net.addInputModule(inp)
        net.addModule(h1)
        net.addModule(h2)

        # Creating the connections
        net.addConnection(FullConnection(inp, h1))
        net.addConnection(FullConnection(h1, h2))
        net.addConnection(FullConnection(h2, outp))
        net.sortModules()

        # Training the neural network using Backpropagation
        t = BackpropTrainer(net, learningrate=0.01, momentum=0.5, verbose=True)
        t.trainOnDataset(dataset, 5)
        t.testOnData(verbose=False)

        # Saving the trained neural network information to file
        self.writetrainedinfo(net)

    def writetrainedinfo(self, neuralnetwork):
        """
        # Using the Python pickle
        fileObject = open('traininfo', 'w')
        pickle.dump(neuralnetwork, fileObject)
        fileObject.close()
        """
        # Writing file using the NetworkWriter
        NetworkWriter.writeToFile(neuralnetwork, 'trainedinfo.xml')

if __name__ == '__main__':
    nn = NeuralNetworkTraining()
    nn.neuralnetworktrain()
