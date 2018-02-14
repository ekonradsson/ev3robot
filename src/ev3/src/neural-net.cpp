#include <vector>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

#include "neural-net.hpp"

using namespace std;

void TrainingData::getTopology(vector<unsigned> &topology)
{
    string line;
    string label;

    if (!m_trainingDataFile.is_open())
    {
        cout << "Could not open training file" << endl;
        exit(-1);
    }
    getline(m_trainingDataFile, line);
    stringstream ss(line);
    ss >> label;
    if(this->isEof() || label.compare("topology:") != 0)
    {
        abort();
    }

    topology.clear();
    while(!ss.eof())
    {
        unsigned n;
        ss >> n;
        topology.push_back(n);
    }
    return;
}

TrainingData::TrainingData(const string filename)
{
    m_trainingDataFile.open(filename.c_str());
}

void TrainingData::reset()
{
    m_trainingDataFile.clear();
    m_trainingDataFile.seekg(0);
}


unsigned TrainingData::getNextInputs(vector<double> &inputVals)
{
    inputVals.clear();

    string line;
    getline(m_trainingDataFile, line);
    stringstream ss(line);

    string label;
    ss >> label;
    if (label.compare("in:") == 0) {
        double oneValue;
        while (ss >> oneValue) {
            inputVals.push_back(oneValue);
        }
    }

    return inputVals.size();
}

unsigned TrainingData::getTargetOutputs(vector<double> &targetOutputVals)
{
    targetOutputVals.clear();

    string line;
    getline(m_trainingDataFile, line);
    stringstream ss(line);

    string label;
    ss>> label;
    if (label.compare("out:") == 0) {
        double oneValue;
        while (ss >> oneValue) {
            targetOutputVals.push_back(oneValue);
        }
    }

    return targetOutputVals.size();
}

void Neuron::updateInputWeights(Layer &prevLayer)
{
    // The weights to be updated are in the Connection container
    // in the nuerons in the preceding layer

    for(unsigned n = 0; n < prevLayer.size(); ++n)
    {
        Neuron &neuron = prevLayer[n];

        double oldDeltaWeight = neuron.m_outputWeights[m_myIndex].deltaWeight;

        double newDeltaWeight =
                // Individual input, magnified by the gradient and train rate:
                learnRate * neuron.getOutputVal() * m_gradient
                // Also add momentum = a fraction of the previous delta weight
                + momentum * oldDeltaWeight
                //Subtract weight decay
                - decay * oldDeltaWeight;

        neuron.m_outputWeights[m_myIndex].deltaWeight = newDeltaWeight;
        neuron.m_outputWeights[m_myIndex].weight += newDeltaWeight;

        if (neuron.m_outputWeights[m_myIndex].weight < -10.0) // restriction
          neuron.m_outputWeights[m_myIndex].weight = -10.0;
        else if (neuron.m_outputWeights[m_myIndex].weight > 10.0)
          neuron.m_outputWeights[m_myIndex].weight = 10.0;
    }
}

void Neuron::getWeights(vector<double> &v)
{
    v.clear();
    for (int i=0; i< m_outputWeights.size();++i)
    {
        v.push_back(m_outputWeights[i].weight);
    }
}

void Neuron::setWeights(const vector<double> &v)
{
    assert(m_outputWeights.size() == v.size());
    for (int i=0; i< m_outputWeights.size();++i)
    {
        m_outputWeights[i].weight = v[i];
    }
}

double Neuron::sumDOW(const Layer &nextLayer) const
{
    double sum = 0.0;

    // Sum our contributions of the errors at the nodes we feed

    for (unsigned n = 0; n < nextLayer.size() - 1; ++n)
    {
        sum += m_outputWeights[n].weight * nextLayer[n].m_gradient;
    }

    return sum;
}

void Neuron::calcHiddenGradients(const Layer &nextLayer)
{
    double dow = sumDOW(nextLayer);
    m_gradient = dow * Neuron::transferFunctionDerivative(m_outputVal);
}

void Neuron::calcOutputGradients(double targetVals)
{
    double delta = targetVals - m_outputVal;
    m_gradient = delta * Neuron::transferFunctionDerivative(m_outputVal);
}

double Neuron::transferFunction(double x)
{
    // tanh - output range [-1.0..1.0]
    if (x < -20.0) return -1.0; // approximation is correct to 30 decimals
    else if (x > 20.0) return 1.0;
    else return tanh(x);
}

double Neuron::transferFunctionDerivative(double x)
{
    // tanh derivative
    return 1.0 - x * x;
    //return (1.0 - x) * (1.0 + x);
}

void Neuron::feedForward(const Layer &prevLayer)
{
    double sum = 0.0;

    // Sum the previous layer's outputs (which are our inputs)
    // Include the bias node from the previous layer.

    for(unsigned n = 0 ; n < prevLayer.size(); ++n)
    {
        sum += prevLayer[n].getOutputVal() *
                prevLayer[n].m_outputWeights[m_myIndex].weight;
    }

    m_outputVal = Neuron::transferFunction(sum);
}

Neuron::Neuron(unsigned numOutputs, unsigned myIndex)
{
    for(unsigned c = 0; c < numOutputs; ++c){
        m_outputWeights.push_back(Connection());
            m_outputWeights.back().weight = randomWeight();
    }

    m_myIndex = myIndex;
}


void Net::getResults(vector<double> &resultVals) const
{
    resultVals.clear();

    for(unsigned n = 0; n < m_layers.back().size() - 1; ++n)
    {
        resultVals.push_back(m_layers.back()[n].getOutputVal());
    }
}

void Net::backProp(const std::vector<double> &targetVals)
{
    Layer &outputLayer = m_layers.back();

    // Calculate overal net error (RMS of output neuron errors)
    m_error = 0.0;

    for(unsigned n = 0; n < outputLayer.size() - 1; ++n)
    {
        double delta = targetVals[n] - outputLayer[n].getOutputVal();
        m_error += delta *delta;
    }
    m_error /= outputLayer.size() - 1; // get average error squared
    m_error = sqrt(m_error); // RMS

    // Implement a recent average measurement:
    m_recentAverageError =
            (m_recentAverageError * m_recentAverageSmoothingFactor + m_error)
            / (m_recentAverageSmoothingFactor + 1.0);


    // Calculate output layer gradients
    for(unsigned n = 0; n < outputLayer.size() - 1; ++n)
    {
        outputLayer[n].calcOutputGradients(targetVals[n]);
    }
    // Calculate gradients on hidden layers

    for(unsigned layerNum = m_layers.size() - 2; layerNum > 0; --layerNum)
    {
        Layer &hiddenLayer = m_layers[layerNum];
        Layer &nextLayer = m_layers[layerNum + 1];

        for(unsigned n = 0; n < hiddenLayer.size(); ++n)
        {
            hiddenLayer[n].calcHiddenGradients(nextLayer);
        }
    }

    // For all layers from outputs to first hidden layer,
    // update connection weights
    for(unsigned layerNum = m_layers.size() - 1; layerNum > 0; --layerNum)
    {
        Layer &layer = m_layers[layerNum];
        Layer &prevLayer = m_layers[layerNum - 1];

        for(unsigned n = 0; n < layer.size() - 1; ++n)
        {
            layer[n].updateInputWeights(prevLayer);
        }
    }
}

void Net::feedForward(const vector<double> &inputVals)
{
    // Check the num of inputVals euqal to neuronnum expect bias
    assert(inputVals.size() == m_layers[0].size() - 1);

    // Assign {latch} the input values into the input neurons
    for(unsigned i = 0; i < inputVals.size(); ++i){
        m_layers[0][i].setOutputVal(inputVals[i]);
    }

    // Forward propagate
    for(unsigned layerNum = 1; layerNum < m_layers.size(); ++layerNum){
        Layer &prevLayer = m_layers[layerNum - 1];
        for(unsigned n = 0; n < m_layers[layerNum].size() - 1; ++n){
            m_layers[layerNum][n].feedForward(prevLayer);
        }
    }
}

void Net::getWeights(std::vector<std::vector<std::vector<double> > > &v)
{
    v.clear();
    for(unsigned layerNum = 0; layerNum < m_layers.size()-1; ++layerNum)
    {
        std::vector<std::vector<double> > layerWeights;
        Layer &layer = m_layers[layerNum];
        for(unsigned n = 0; n < layer.size(); ++n)
        {
            vector<double> nodeWeights;
            layer[n].getWeights(nodeWeights);
            layerWeights.push_back(nodeWeights);
        }
        v.push_back(layerWeights);
    }
}

void Net::setWeights(const std::vector<std::vector<std::vector<double> > > &v)
{
    assert( m_layers.size()-1 == v.size());
    for(unsigned layerNum = 0; layerNum < m_layers.size()-1; ++layerNum)
    {
        Layer &layer = m_layers[layerNum];
        assert( layer.size() == v[layerNum].size());
        for(unsigned n = 0; n < layer.size(); ++n)
        {
            layer[n].setWeights(v[layerNum][n]);
        }
    }
}


Net::Net(const vector<unsigned> &topology)
{
    unsigned numLayers = topology.size();
    for(unsigned layerNum = 0; layerNum < numLayers; ++layerNum){
        m_layers.push_back(Layer());
        // numOutputs of layer[i] is the numInputs of layer[i+1]
        // numOutputs of last layer is 0
        unsigned numOutputs = layerNum == topology.size() - 1 ? 0 :topology[layerNum + 1];

        // We have made a new Layer, now fill it ith neurons, and
        // add a bias neuron to the layer:

        for(unsigned neuronNum = 0; neuronNum <= topology[layerNum]; ++neuronNum){
            m_layers.back().push_back(Neuron(numOutputs, neuronNum));
        }

        // Force the bias node's output value to 1.0. It's the last neuron created above
        m_layers.back().back().setOutputVal(1.0);
    }
}

