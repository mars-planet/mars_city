from pybrain.datasets import ClassificationDataSet
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.tools.shortcuts import buildNetwork
from pybrain.tools.customxml.networkwriter import NetworkWriter
from pybrain.structure.networks import FeedForwardNetwork
import csv


class XTraining(object):

    def getdata(self):
        dataset = ClassificationDataSet(9, 1)
        with open('xtraindata.csv') as tf:
            for line in tf:
                data = [x for x in line.strip().split(',') if x]
                # indata =  tuple(data[1:10])
                # outdata = tuple(data[10:])
                """
                for i in range(4,10):
                    data[i] = str(float(data[i])*100)
                if float(data[12]) > 0:
                    data[12] = float(data[12]) * 100
                """
                for i in range(1, 4):
                    data[i] = str(float(data[i]) / 100)
                dataset.appendLinked(data[1:10], data[12])
        tf.close()
        return dataset

    def xtrain(self):
        dataset = self.getdata()

        # Constructing a two hidden layes Neural Network
        net = buildNetwork(9, 15, 5, 1, recurrent=True)

        # Training using Back Propagation
        trainer = BackpropTrainer(net, learningrate=0.01, momentum=0.75,
                                  weightdecay=0.02, verbose=True)
        trainer.trainOnDataset(dataset, 10)
        trainer.testOnData(verbose=False)

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
        NetworkWriter.writeToFile(neuralnetwork, 'xtrainedinfo.xml')

if __name__ == '__main__':
    nn = XTraining()
    nn.xtrain()
